#ifndef __LAN91C111_H
#define __LAN91C111_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "PeripheralNames.h"

#define FVP_ETH_MAX_FLEN         (1522) // recommended size for a VLAN frame
#define FVP_HWADDR_SIZE           (6)

#define FVP_ETH_MTU_SIZE          1500
#define FVP_ETH_IF_NAME           "fm"

#define LAN91_BUFF_ALIGNMENT 16U   /*!< Ethernet buffer alignment. */

/*--------------------------- End of function ---------------------------------*/


#define ADROFS     0x40200000   /* LAN91C111 base address used IO mode      */

/* Absolute access to LAN91C111 registers macro */
#define LREG(object, reg)   (*((object volatile *) (ADROFS+reg)))

/* Bank Select defines */
#define BSR        0x0E         /* Bank Select register common to all banks  */
#define BSR_MASK   0x03         /* Mask constant part of bank register       */
#define BSR_UPPER  0x3300       /* Constant value for upper byte of BSR      */

/* Bank 0 Registers */
#define B0_TCR     0x00         /* Transmit Control Register           rd/wr */
#define B0_EPHSR   0x02         /* EPH Status Register                 rd    */
#define B0_RCR     0x04         /* Receive Control Register            rd/wr */
#define B0_ECR     0x06         /* Counter Register                    rd    */
#define B0_MIR     0x08         /* Memory Information Register         rd    */
#define B0_RPCR    0x0A         /* Receive/Phy Control Register        rd/wr */
#define B0_RES     0x0C         /* Reserved                                  */

/* Bank 1 Registers */
#define B1_CR      0x00         /* Configuration Register              rd/wr */
#define B1_BAR     0x02         /* Base Address Register               rd/wr */
#define B1_IAR     0x04         /* Individual Address Registers        rd/wr */
#define B1_IAR0    0x04         /* Individual Address Bytes 0-1        rd/wr */
#define B1_IAR2    0x06         /* Individual Address Bytes 2-3        rd/wr */
#define B1_IAR4    0x08         /* Individual Address Bytes 4-5        rd/wr */
#define B1_GPR     0x0A         /* General Purpose Register            rd/wr */
#define B1_CTR     0x0C         /* Control Register                    rd/wr */

/* Bank 2 Registers */
#define B2_MMUCR   0x00         /* MMU Command Register                rd/wr */
#define B2_PNR     0x02         /* Packet Number Register (8 bit)      rd/wr */
#define B2_ARR     0x03         /* Allocation Result Register (8 bit)  rd    */
#define B2_FIFO    0x04         /* FIFO Ports Register                 rd    */
#define B2_TX_FIFO 0x04         /* Tx FIFO Packet Number (8 bit)       rd    */
#define B2_RX_FIFO 0x05         /* Rx FIFO Packet Number (8 bit)       rd    */
#define B2_PTR     0x06         /* Pointer Register                    rd/wr */
#define B2_DATA    0x08         /* Data Register (8/16/32 bit)         rd/wr */
#define B2_DATA0   0x08         /* Data Register Word 0                rd/wr */
#define B2_DATA1   0x0A         /* Data Register Word 1                rd/wr */
#define B2_IST     0x0C         /* Interrupt Status Register (8 bit)   rd    */
#define B2_ACK     0x0C         /* Interrupt Ack Register (8 bit)         wr */
#define B2_MSK     0x0D         /* Interrupt Mask Register (8 bit)     rd/wr */

/* Bank 3 Registers */
#define B3_MT      0x00         /* Multicast Hash Table                rd/wr */
#define B3_MT0     0x00         /* Multicast Hash Table 0-1            rd/wr */
#define B3_MT2     0x02         /* Multicast Hash Table 2-3            rd/wr */
#define B3_MT4     0x04         /* Multicast Hash Table 4-5            rd/wr */
#define B3_MT6     0x06         /* Multicast Hash Table 6-7            rd/wr */
#define B3_MGMT    0x08         /* Management Interface PHY            rd/wr */
#define B3_REV     0x0A         /* Revision Register (Chip Id/Revision)rd/wr */
#define B3_ERCV    0x0C         /* Early Receive Register              rd/wr */


/*-----TCR control bits-----*/
#define TCR_SWFDUP      0x8000  /* Switched Full Duplex Mode                 */
#define TCR_EPH_LOOP    0x2000  /* Internal Loopback at the EPH block        */
#define TCR_STP_SQET    0x1000  /* Stop transmit on SQET error               */
#define TCR_FDUPLX      0x0800  /* Full duplex mode (receive own frames)     */
#define TCR_MON_CSN     0x0400  /* Monitor carrier while transmitting        */
#define TCR_NOCRC       0x0100  /* Don't append CRC to tx frames             */
#define TCR_PAD_EN      0x0080  /* Pad short frames with 0 if len < 64 bytes */
#define TCR_FORCOL      0x0004  /* Force collision                           */
#define TCR_LOOP        0x0002  /* PHY Local loopback                        */
#define TCR_TXENA       0x0001  /* Enable transmitter                        */

/*-----EPHSR status bits-----*/
#define EPHSR_TXUNRN    0x8000  /* Transmit Under Run                        */
#define EPHSR_LINK_OK   0x4000  /* General purpose input driven by nLNK pin  */
#define EPHSR_CTR_ROL   0x1000  /* Counter Roll Over                         */
#define EPHSR_EXC_DEF   0x0800  /* Excessive Deferral                        */
#define EPHSR_LOST_CARR 0x0400  /* Lost Carrier Sense                        */
#define EPHSR_LATCOL    0x0200  /* Late Collision Detected                   */
#define EPHSR_TX_DEFR   0x0080  /* Transmit Deferred                         */
#define EPHSR_LTX_BRD   0x0040  /* Last Tx Frame was a broadcast             */
#define EPHSR_SQET      0x0020  /* Signal Quality Error Test                 */
#define EPHSR_16COL     0x0010  /* 16 collisions reached                     */
#define EPHSR_LTX_MULT  0x0008  /* Last transmit frame was a multicast       */
#define EPHSR_MULCOL    0x0004  /* Multiple collision detected for last tx   */
#define EPHSR_SNGLCOL   0x0002  /* Single collision detected for last tx     */
#define EPHSR_TX_SUC    0x0001  /* Last transmit was successful              */

/*-----RCR control bits-----*/
#define RCR_SOFT_RST    0x8000  /* Software-Activated Reset                  */
#define RCR_FILT_CAR    0x4000  /* Filter Carrier                            */
#define RCR_ABORT_ENB   0x2000  /* Enable Abort of Rx when collision         */
#define RCR_STRIP_CRC   0x0200  /* Strip CRC of received frames              */
#define RCR_RXEN        0x0100  /* Enable Receiver                           */
#define RCR_ALMUL       0x0004  /* Accept all multicast (no filtering)       */
#define RCR_PRMS        0x0002  /* Promiscuous mode                          */
#define RCR_RX_ABORT    0x0001  /* Receive frame aborted (too long)          */

/*-----RPCR control bits-----*/
#define RPCR_SPEED      0x2000  /* Speed select input (10/100 MBps)          */
#define RPCR_DPLX       0x1000  /* Duplex Select (Full/Half Duplex)          */
#define RPCR_ANEG       0x0800  /* Auto-Negotiation mode select              */
#define RPCR_LEDA_MASK  0x00E0  /* LEDA signal mode select                   */
#define RPCR_LEDB_MASK  0x001C  /* LEDB signal mode select                   */

/*-----LEDA ON modes-----*/
#define LEDA_10M_100M   0x0000  /* 10 MB or 100 MB link detected             */
#define LEDA_10M        0x0040  /* 10 MB link detected                       */
#define LEDA_FDUPLX     0x0060  /* Full Duplex Mode enabled                  */
#define LEDA_TX_RX      0x0080  /* Transmit or Receive packet occurred       */
#define LEDA_100M       0x00A0  /* 100 MB link detected                      */
#define LEDA_RX         0x00C0  /* Receive packet occurred                   */
#define LEDA_TX         0x00E0  /* Transmit packet occurred                  */

/*-----LEDA ON modes-----*/
#define LEDB_10M_100M   0x0000  /* 10 MB or 100 MB link detected             */
#define LEDB_10M        0x0008  /* 10 MB link detected                       */
#define LEDB_FDUPLX     0x000C  /* Full Duplex Mode enabled                  */
#define LEDB_TX_RX      0x0010  /* Transmit or Receive packet occurred       */
#define LEDB_100M       0x0014  /* 100 MB link detected                      */
#define LEDB_RX         0x0018  /* Receive packet occurred                   */
#define LEDB_TX         0x001C  /* Transmit packet occurred                  */

/*-----CR control bits-----*/
#define CR_EPH_POW_EN   0x8000  /* EPH Power Enable  (0= power down PHY)     */
#define CR_NO_WAIT      0x1000  /* No wait states                            */
#define CR_GPCNTRL      0x0400  /* General purpose Output drives nCNTRL pin  */
#define CR_EXT_PHY      0x0200  /* External PHY enabled (0= internal PHY)    */
#define CR_DEFAULT      0x20B1  /* Default bits set to 1 for write           */

/*-----CTR control bits-----*/
#define CTR_RCV_BAD     0x4000  /* Bad CRC packet received                   */
#define CTR_AUTO_REL    0x0800  /* Auto-release Tx memory                    */
#define CTR_LE_ENABLE   0x0080  /* Link error enable (mux into EPH int)      */
#define CTR_CR_ENABLE   0x0040  /* Counter rollover enable (mux into EPH int)*/
#define CTR_TE_ENABLE   0x0020  /* Transmit error enable (mux into EPH int)  */
#define CTR_EEPROM_SEL  0x0004  /* EEPROM select                             */
#define CTR_RELOAD      0x0002  /* Reload from EEPROM                        */
#define CTR_STORE       0x0001  /* Store to EEPROM                           */
#define CTR_DEFAULT     0x1210  /* Default bits set to 1 for write           */

/*-----MMUCR control bits-----*/
#define MMUCR_CMD_MASK  0x00E0  /* MMU Command mask                          */
#define MMUCR_BUSY      0x0001  /* MMU processing a release command          */

/*-----MMUCR Commands------*/
#define MMU_NOOP        0x0000  /* No operation                              */
#define MMU_ALLOC_TX    0x0020  /* Allocate memory for Tx                    */
#define MMU_RESET       0x0040  /* Reset MMU to initial state                */
#define MMU_REMV_RX     0x0060  /* Remove frame from top of Rx FIFO          */
#define MMU_REMV_REL_RX 0x0080  /* Remove and Release top of Rx FIFO         */
#define MMU_REL_PKT     0x00A0  /* Release specific packet                   */
#define MMU_ENQ_TX      0x00C0  /* Enqueue packet number into Tx FIFO        */
#define MMU_RESET_TX    0x00E0  /* Reset Tx FIFO                             */

/*-----FIFO status bits-----*/
#define FIFO_REMPTY     0x8000  /* No receive packets queued in Rx FIFO      */
#define FIFO_TEMPTY     0x0080  /* No transmit packets in completion queue   */

/*-----PTR control bits-----*/
#define PTR_RCV         0x8000  /* Address refers to Rx area (0= Tx area)    */
#define PTR_AUTO_INCR   0x4000  /* Auto increment on access                  */
#define PTR_READ        0x2000  /* Read access (0= write access)             */
#define PTR_ETEN        0x1000  /* Enable early transmit underrun detection  */
#define PTR_NOT_EMPTY   0x0800  /* Data FIFO not empty yet (read only bit)   */
#define PTR_MASK        0x03FF  /* Mask pointer value                        */

/*-----IST status bits-----*/
#define IST_MDINT       0x80    /* PHY MI Register 18 change status interrupt*/
#define IST_ERCV_INT    0x40    /* Early Receive interrupt                   */
#define IST_EPH_INT     0x20    /* EPH Type interrupt                        */
#define IST_RX_OVRN     0x10    /* Receive Overrun interrupt                 */
#define IST_ALLOC_INT   0x08    /* Tx ram Allocation interrupt               */
#define IST_TX_EMPTY    0x04    /* Tx FIFO empty interrupt                   */
#define IST_TX_INT      0x02    /* Tx Complete interrupt                     */
#define IST_RCV         0x01    /* Rx Complete intererupt                    */

/*-----ACK control bits-----*/
#define ACK_MDINT       0x80    /* PHY MI Register 18 change int. ack        */
#define ACK_ERCV_INT    0x40    /* Early Receive int. ack                    */
#define ACK_RX_OVRN     0x10    /* Receive Overrun int. ack                  */
#define ACK_TX_EMPTY    0x04    /* Tx FIFO empty int. ack                    */
#define ACK_TX_INT      0x02    /* Tx Complete int. ack                      */

/*-----MSK control bits-----*/
#define MSK_MDINT       0x80    /* PHY MI Register 18 change int. mask       */
#define MSK_ERCV_INT    0x40    /* Early Receive int. mask                   */
#define MSK_EPH_INT     0x20    /* EPH Type int. mask                        */
#define MSK_RX_OVRN     0x10    /* Receive Overrun int. mask                 */
#define MSK_ALLOC_INT   0x08    /* Tx ram Allocation int. mask               */
#define MSK_TX_EMPTY    0x04    /* Tx FIFO empty int. mask                   */
#define MSK_TX_INT      0x02    /* Tx Complete int. mask                     */
#define MSK_RCV         0x01    /* Rx Complete int. mask                     */

/*-----MGMT control bits-----*/
#define MGMT_MSK_CRS100 0x0040  /* Disables CRS100 detection in Tx Half Dupl.*/
#define MGMT_MDOE       0x0008  /* MII - 1= MDO pin output, 0= MDO tristated */
#define MGMT_MCLK       0x0004  /* MII - Value drives MDCLK pin              */
#define MGMT_MDI        0x0002  /* MII - Value of MDI pin when read          */
#define MGMT_MDO        0x0001  /* MII - Value drives MDO pin                */
#define MGMT_DEFAULT    0x3330  /* Default bits set to 1 for write           */

/*----- Receive Frame Status -----*/
#define RFS_ALGNERR     0x8000  /* Frame alignment error                     */
#define RFS_BROADCAST   0x4000  /* Received broadcast frame                  */
#define RFS_BADCRC      0x2000  /* Bad CRC error                             */
#define RFS_ODDFRM      0x1000  /* Frame with Odd number of bytes received   */
#define RFS_TOOLNG      0x0800  /* Too long frame received (max. 1518 bytes) */
#define RFS_TOOSHORT    0x0400  /* Too short frame received (min. 64 bytes)  */
#define RFS_MULTCAST    0x0001  /* Multicast frame received                  */
#define RFS_HASH_MASK   0x007E  /* Hash value index for multicast registers  */

/*----- Receive Frame Control -----*/
#define RFC_ODD         0x2000  /* Odd number of bytes in frame              */
#define RFC_CRC         0x1000  /* Append CRC (valid when TCR_NOCRC = 1)     */

#endif



/*----------------------------------------------------------------------------*/
/* Stractures */

/*! @brief Defines the status return codes for transaction. */
enum _lan91_status
{
    LAN91_Success,
    LAN91_RxFrameError, /*!< A frame received but data error happen. */
    LAN91_RxFrameFail,  /*!< Failed to receive a frame. */
    LAN91_RxFrameEmpty, /*!< No frame arrive. */
    LAN91_TxFrameBusy,                       /*!< Transmit buffer descriptors are under process. */
    LAN91_TxFrameFail /*!< Transmit frame fail. */
};


/*! @brief List of interrupts supported by the peripheral. This
 * enumeration uses one-bot encoding to allow a logical OR of multiple
 * members. Members usually map to interrupt enable bits in one or more
 * peripheral registers.
 */
typedef enum _lan91_phy_status
{
    STATE_UNKNOWN    = (-1),    /* PHY MI Register 18 change status interrupt*/
    STATE_LINK_DOWN  = (0),    /* EPH Type interrupt                        */
    STATE_LINK_UP  = (1)    /* Receive Overrun interrupt                 */
} lan91_phy_status_t;

/*! @brief Defines the common interrupt event for callback use. */
typedef enum _lan91_event
{
    LAN91_RxEvent,     /*!< Receive event. */
    LAN91_TxEvent,     /*!< Transmit event. */
    LAN91_ErrEvent    /*!< Error event: BABR/BABT/EBERR/LC/RL/UN/PLR . */
} lan91_event_t;


/* ------------------------------- Call back ---------------------------------*/
/* Forward declaration of the handle typedef. */
typedef struct _lan91_handle lan91_handle_t;


/*! @brief ENET callback function. */
typedef void (*lan91_callback_t)( lan91_event_t event, void *userData);


/*! @brief Defines the ENET handler structure. */
struct _lan91_handle
{
    lan91_callback_t callback;                  /*!< Callback function. */
    void *userData;                            /*!< Callback function parameter.*/
};


/*--------------------------- APIs ----------------------------------*/
#if defined(__cplusplus)
extern "C" {
#endif


/* ------------------- Function Prototypes ------------------------------*/
void LAN91_init (void);

bool LAN91_send_frame (uint32_t *buff, uint32_t *size );
bool LAN91_receive_frame (uint32_t *buff, uint32_t *size );

void ETHERNET_Handler(void);

static void delay_1ms (uint32_t time);
static void output_MDO (int bit_value);
static void output_MDO (int bit_value);
static int  input_MDI (void);
static void write_PHY (uint32_t PhyReg, int Value);
static uint16_t  read_PHY (uint32_t PhyReg);



/*!
 * @name Transactional operation
 * @{
 */

/*!
 * @brief Sets the callback function.
 * This API is provided for the application callback required case when ENET
 * interrupt is enabled. This API should be called after calling ENET_Init.
 *
 * @param handle ENET handler pointer. Should be provided by application.
 * @param callback The ENET callback function.
 * @param userData The callback function parameter.
 */
void LAN91_SetCallback(lan91_callback_t callback, void *userData);



/*--------------------------- inline functions ---------------------------------*/

/*!
 * @brief Resets the ENET module.
 *
 * This function restores the ENET module to the reset state.
 * Note that this function sets all registers to the
 * reset state. As a result, the ENET module can't work after calling this function.
 *
 * @param base  ENET peripheral base address.
 */
static inline void LAN91_Reset(void)
{


    /* Output reset to LAN controller */
    LREG (uint16_t, BSR)    = 0;
    LREG (uint16_t, B0_RCR) = RCR_SOFT_RST;
}


/*!
 * @name Other basic operations
 * @{
 */

/*!
 * @brief Activates ENET read or receive.
 *
 * @param base  ENET peripheral base address.
 *
 * @note This must be called after the MAC configuration and
 * state are ready. It must be called after the ENET_Init() and
 * ENET_Ptp1588Configure(). This should be called when the ENET receive required.
 */
static inline void LAN91_ActiveRead()
{
    // add code
}

/*!
 * @brief Gets the ENET interrupt status flag.
 *
 * @param base  ENET peripheral base address.
 * @return The event status of the interrupt source. This is the logical OR of members
 *         of the enumeration :: enet_interrupt_enable_t.
 */
static inline uint8_t LAN91_GetInterruptStatus(void)
{
    /* Select BANK 2 */
    LREG (uint16_t, BSR) = 2;
    return LREG (uint8_t, B2_IST);
}

static inline bool LAN91_GetFIFOStatus(void)
{
    /* Select BANK 2 */
    LREG (uint16_t, BSR) = 2;
    return (( LREG (uint16_t, B2_FIFO) & FIFO_REMPTY ) == 0);
}

/*!
 * @brief Clears the ENET interrupt events status flag.
 *
 * This function clears enabled ENET interrupts according to the provided mask. The mask
 * is a logical OR of enumeration members. See the @ref enet_interrupt_enable_t.
 * For example, to clear the TX frame interrupt and RX frame interrupt, do the following.
 * @code
 *     ENET_ClearInterruptStatus(ENET, kENET_TxFrameInterrupt | kENET_RxFrameInterrupt);
 * @endcode
 *
 * @param base  ENET peripheral base address.
 * @param mask  ENET interrupt source to be cleared.
 * This is the logical OR of members of the enumeration :: enet_interrupt_enable_t.
 */
static inline void LAN91_ClearInterruptStatus(void)
{
    /* Mask off all interrupts */
    LREG (uint16_t, BSR)    = 2;
    LREG (uint8_t,  B2_MSK) = 0;
}


static inline void LAN91_SetHandler(void)
{
    NVIC_EnableIRQ(ETHERNET_IRQn);
}


/*--------------------------- End of inline functions ---------------------------------*/



#if defined(__cplusplus)
}
#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

