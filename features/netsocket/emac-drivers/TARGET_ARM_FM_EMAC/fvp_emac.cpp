#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "cmsis_os.h"

#include "mbed_interface.h"
#include "mbed_assert.h"
#include "netsocket/nsapi_types.h"
#include "mbed_shared_queues.h"


#include "fvp_emac.h"

#define THREAD_STACKSIZE          16384

/********************************************************************************
 * Internal data
 ********************************************************************************/

/* \brief Flags for worker thread */
#define FLAG_TX  1
#define FLAG_RX  2

/** \brief  Driver thread priority */
#define THREAD_PRIORITY (osPriorityNormal)

#define PHY_TASK_PERIOD_MS      200


/* Constructor */
fvp_EMAC::fvp_EMAC()
{
}



/** \brief  Create a new thread for TX/RX.
 */
static osThreadId_t create_new_thread(const char *threadName, void (*thread)(void *arg), void *arg, int stacksize, osPriority_t priority, mbed_rtos_storage_thread_t *thread_cb)
{
    osThreadAttr_t attr = {0};
    attr.name = threadName;
    attr.stack_mem  = malloc(stacksize);
    attr.cb_mem  = thread_cb;
    attr.stack_size = stacksize;
    attr.cb_size = sizeof(mbed_rtos_storage_thread_t);
    attr.priority = priority;
    return osThreadNew(thread, arg, &attr);
}



void fvp_EMAC::ethernet_callback(lan91_event_t event, void *param)
{
    fvp_EMAC *enet = static_cast<fvp_EMAC *>(param);
    switch (event)
    {
      case LAN91_RxEvent:
        enet->rx_isr();
        break;
      case LAN91_TxEvent:
        enet->tx_isr();
        break;
      default:
        break;
    }
}

/** \brief Ethernet receive interrupt handler
 *
 *  This function handles the receive interrupt.
 */
void fvp_EMAC::rx_isr()
{
    if (thread) {
        osThreadFlagsSet(thread, FLAG_RX);
    }
}

void fvp_EMAC::tx_isr()
{
    osThreadFlagsSet(thread, FLAG_TX);
}


/** \brief  Low level init of the MAC and PHY.
 */
bool fvp_EMAC::low_level_init_successful()
{
    LAN91_init();
    LAN91_SetCallback(&fvp_EMAC::ethernet_callback, this);
    LAN91_ActiveRead();
    return true;
}


/** \brief  Worker thread.
 *
 * Woken by thread flags to receive packets or clean up transmit
 *
 *  \param[in] pvParameters pointer to the interface data
 */
void fvp_EMAC::thread_function(void* pvParameters)
{
    struct fvp_EMAC *fvp_enet = static_cast<fvp_EMAC *>(pvParameters);

    for (;;) {
        uint32_t flags = osThreadFlagsWait(FLAG_RX|FLAG_TX, osFlagsWaitAny, osWaitForever);

        MBED_ASSERT(!(flags & osFlagsError));

        if (flags & FLAG_RX) {
            fvp_enet->packet_rx();
        }

        if (flags & FLAG_TX) {
            fvp_enet->packet_tx();
        }
    }
}


/** \brief  Packet reception task
 *
 * This task is called when a packet is received. It will
 * pass the packet to the LWIP core.
 */
void fvp_EMAC::packet_rx()
{

    while(LAN91_GetFIFOStatus())
    {
    	// printf(">>>> RX FIFO Receiving\n");
        emac_mem_buf_t *temp_rxbuf = NULL;
        uint32_t *rx_payload_ptr;
        uint32_t rx_length = 0;

        temp_rxbuf = memory_manager->alloc_heap(FVP_ETH_MAX_FLEN, LAN91_BUFF_ALIGNMENT);

        /* no memory been allocated*/
        if (NULL != temp_rxbuf) {

#ifdef LOCK_RX_THREAD
    /* Get exclusive access */
    TXLockMutex.lock();
#endif
            rx_payload_ptr = (uint32_t*)memory_manager->get_ptr(temp_rxbuf);
            rx_length = memory_manager->get_len(temp_rxbuf);
            // printf(">>>> RX buffer length before %d\n", rx_length);
            
            if (!LAN91_receive_frame(rx_payload_ptr, &rx_length))
            {
#ifdef LOCK_RX_THREAD
TXLockMutex.unlock();
#endif
            // printf(">>>> RX Failed\n");
            break;


            }
            else{
            	// printf(">>>> RX buffer length after %d\n", rx_length);
                memory_manager->set_len(temp_rxbuf, rx_length);
            }
            

#ifdef LOCK_RX_THREAD
    osMutexRelease(TXLockMutex);
#endif

            emac_link_input_cb(temp_rxbuf);

        }
    }
    LREG (uint16_t, BSR) = 2;
    LREG (uint8_t,  B2_MSK) = MSK_RX_OVRN | MSK_RCV;
}


/** \brief  Transmit cleanup task
 *
 * This task is called when a transmit interrupt occurs and
 * reclaims the buffer and descriptor used for the packet once
 * the packet has been transferred.
 */
void fvp_EMAC::packet_tx()
{
    tx_reclaim();
}

/** \brief  Free TX buffers that are complete
 */
void fvp_EMAC::tx_reclaim()
{
  /* Get exclusive access */
  TXLockMutex.lock();

    // Free package
    // memory_manager->free(tx_buff[tx_consume_index % ENET_TX_RING_LEN]);

  /* Restore access */
  TXLockMutex.unlock();
}



/** \brief  Low level output of a packet. Never call this from an
 *          interrupt context, as it may block until TX descriptors
 *          become available.
 *
 *  \param[in] buf      the MAC packet to send (e.g. IP packet including MAC addresses and type)
 *  \return ERR_OK if the packet could be sent or an err_t value if the packet couldn't be sent
 */
bool fvp_EMAC::link_out(emac_mem_buf_t *buf)
{
    // If buffer is chained or not aligned then make a contiguous aligned copy of it
    if (memory_manager->get_next(buf) ||
        reinterpret_cast<uint32_t>(memory_manager->get_ptr(buf)) % LAN91_BUFF_ALIGNMENT) {
        emac_mem_buf_t *copy_buf;
        copy_buf = memory_manager->alloc_heap(memory_manager->get_total_len(buf), LAN91_BUFF_ALIGNMENT);
        if (NULL == copy_buf) {
            memory_manager->free(buf);
            // printf(">>>> TX buffer failed\n");
            return false;
        }

        // Copy to new buffer and free original
        memory_manager->copy(copy_buf, buf);
        memory_manager->free(buf);
        buf = copy_buf;
    }

    /* Check if a descriptor is available for the transfer (wait 10ms before dropping the buffer) */
    //if (xTXDCountSem.wait(10) == 0) {
    //    memory_manager->free(buf);
    //    return false;
    //}

    /* Save the buffer so that it can be freed when transmit is done */
    // tx_buff[tx_produce_index % ENET_TX_RING_LEN] = buf;
    uint32_t * buffer;
    uint32_t tx_length = 0;
    bool state;
    buffer = (uint32_t *)(memory_manager->get_ptr(buf));
    tx_length = memory_manager->get_len(buf);

    // printf(">>>> TX buffer length before %d\n", tx_length);

    /* Get exclusive access */
    TXLockMutex.lock();

    /* Setup transfers */
    state = LAN91_send_frame(buffer,&tx_length);
   
    /* Restore access */
    TXLockMutex.unlock();

    if(!state){
    	// printf(">>>> TX Failed\n");
        return false;
    }
    // printf(">>>> TX buffer length after %d\n", tx_length);
    // this should be freed at tx_reclaim incase of sending error
    memory_manager->free(buf);
    return true;
}

/*******************************************************************************
 * PHY task: monitor link
*******************************************************************************/

// #define STATE_UNKNOWN           (-1)
// #define STATE_LINK_DOWN         (0)
// #define STATE_LINK_UP           (1)

void fvp_EMAC::phy_task()
{
    uint32_t phyAddr = 0;

    // Get current status

    lan91_phy_status_t connection_status;
    // PHY_GetLinkStatus(&connection_status);
    connection_status = STATE_LINK_UP;
    // Compare with previous state
    if (connection_status != prev_state && emac_link_state_cb) {
        emac_link_state_cb(connection_status);
    }

    prev_state = connection_status;
}

bool fvp_EMAC::power_up()
{
    /* Initialize the hardware */
    if (!low_level_init_successful()) {
        return false;
    }

    /* ethernet Worker thread */
    thread = create_new_thread("FVP_EMAC_thread", &fvp_EMAC::thread_function, this, THREAD_STACKSIZE, THREAD_PRIORITY, &thread_cb);

    /* Trigger thread to deal with any RX packets that arrived before thread was started ??? */
    rx_isr();

    /* PHY monitoring task */
    prev_state = STATE_LINK_DOWN;


    mbed::mbed_event_queue()->call(mbed::callback(this, &fvp_EMAC::phy_task));

    /* Allow the PHY task to detect the initial link state and set up the proper flags */
    osDelay(10);

    phy_task_handle = mbed::mbed_event_queue()->call_every(PHY_TASK_PERIOD_MS, mbed::callback(this, &fvp_EMAC::phy_task));

    return true;
}

uint32_t fvp_EMAC::get_mtu_size() const
{
    return FVP_ETH_MTU_SIZE;
}

uint32_t fvp_EMAC::get_align_preference() const
{
    return LAN91_BUFF_ALIGNMENT;
}

void fvp_EMAC::get_ifname(char *name, uint8_t size) const
{
    memcpy(name, FVP_ETH_IF_NAME, (size < sizeof(FVP_ETH_IF_NAME)) ? size : sizeof(FVP_ETH_IF_NAME));
}

uint8_t fvp_EMAC::get_hwaddr_size() const
{
    return FVP_HWADDR_SIZE;
}

bool fvp_EMAC::get_hwaddr(uint8_t *addr) const
{
    addr[0] = 0xaa;
    addr[1] = 0xbb;
    addr[2] = 0xcc;
    addr[3] = 0xdd;
    addr[4] = 0xee;
    addr[5] = 0xff;
    return true;
}

void fvp_EMAC::set_hwaddr(const uint8_t *addr)
{
    // add code
    // // memcpy(hwaddr, addr, sizeof hwaddr);
    // // ENET_SetMacAddr(ENET, const_cast<uint8_t*>(addr));
}

void fvp_EMAC::set_link_input_cb(emac_link_input_cb_t input_cb)
{
    emac_link_input_cb = input_cb;
}

void fvp_EMAC::set_link_state_cb(emac_link_state_change_cb_t state_cb)
{
    emac_link_state_cb = state_cb;
}

void fvp_EMAC::add_multicast_group(const uint8_t *addr)
{
    /* No-op at this stage */
}

void fvp_EMAC::remove_multicast_group(const uint8_t *addr)
{
    /* No-op at this stage */
}

void fvp_EMAC::set_all_multicast(bool all)
{
    /* No-op at this stage */
}

void fvp_EMAC::power_down()
{
    /* No-op at this stage */
}

void fvp_EMAC::set_memory_manager(EMACMemoryManager &mem_mngr)
{
    memory_manager = &mem_mngr;
}


fvp_EMAC &fvp_EMAC::get_instance() {
    static fvp_EMAC emac;
    return emac;
}

// Weak so a module can override
MBED_WEAK EMAC &EMAC::get_default_instance() {
    return fvp_EMAC::get_instance();
}

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */

