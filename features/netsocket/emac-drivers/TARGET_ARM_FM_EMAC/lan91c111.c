/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    LAN91C111.C
 *      Purpose: Driver for SMSC LAN91C111 Ethernet Controller
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

//#include <Net_Config.h>
#include "lan91c111.h"
#include <stddef.h>
//#include <LPC21xx.H>                          /* LPC21xx definitions         */

/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.     */

/* Net_Config.c */
//extern uint8_t own_hw_adr[];

/* Local variables */

/*----------------------------------------------------------------------------
 *      LAN91C111 Ethernet Driver Functions
 *----------------------------------------------------------------------------
 *  Required functions for Ethernet driver module:
 *  a. Polling mode: - void init_ethernet ()
 *                   - void send_frame (OS_FRAME *frame)
 *                   - void poll_ethernet (void)
 *  b. Interrupt mode: - void init_ethernet ()
 *                     - void send_frame (OS_FRAME *frame)
 *                     - void int_enable_eth ()
 *                     - void int_disable_eth ()
 *                     - interrupt function 
 *---------------------------------------------------------------------------*/



lan91_handle_t *lan91c111_handle;


/*--------------------------- init_ethernet ---------------------------------*/

void LAN91_init (void) {
  /* Initialize the Lan91C111 ethernet controller. */
  
  uint32_t tcr,stat;
  int i;

  LAN91_ClearInterruptStatus();
  /* Reset LAN91 module. */
  LAN91_Reset();

  /* Wait 50 ms for PHY to reset. */
  delay_1ms (50); // replace with mbed waitms() or this is not necessary for Models ????

  /* Clear control registers. */
  LREG (uint16_t, BSR)    = 0;
  LREG (uint16_t, B0_RCR) = 0;
  LREG (uint16_t, B0_TCR) = 0;

  /* Read MAC address stored to external EEPROM */
  LREG (uint16_t, BSR) = 1;

  i = LREG (uint16_t, B1_IAR0);
  //own_hw_adr[0] = i;
  //own_hw_adr[1] = i >> 8;
  i = LREG (uint16_t, B1_IAR2);
  //own_hw_adr[2] = i;
  //own_hw_adr[3] = i >> 8;
  i = LREG (uint16_t, B1_IAR4);
  //own_hw_adr[4] = i;
  //own_hw_adr[5] = i >> 8;

  /* Write Configuration Registers */
  LREG (uint16_t, B1_CR) = CR_EPH_POW_EN | CR_DEFAULT;

  /* Wait 50 ms for MMU operation to finish. */
  delay_1ms (50); // replace with mbed waitms() or this is not necessary for Models ????

  /* Establish the link */

  /* Reset the PHY, timeout is 3 sec */
  write_PHY (0, 0x8000);
  for (i = 0; i < 30; i++) {
    delay_1ms (100); // replace with mbed waitms() or this is not necessary for Models ????
    if (!(read_PHY (0) & 0x8000)) {
      /* reset complete */
      break;
    }
  }

  /* Before auto negotiation, clear phy 18 status */
  read_PHY (18);

  /* Set the MAC Register and select the speed. */
  LREG (uint16_t, BSR)     = 0;
#if defined (_10MBIT_)
  /* Connect at 10MBit */
  LREG (uint16_t, B0_RPCR) = LEDA_10M_100M | LEDB_TX_RX;
#elif defined (_100MBIT_)
  /* Connect at 100MBit */
  LREG (uint16_t, B0_RPCR) = RPCR_SPEED | LEDA_10M_100M | LEDB_TX_RX;
#else
  /* Use 100MBit link speed. */
  LREG (uint16_t, B0_RPCR) = RPCR_SPEED | LEDA_10M_100M | LEDB_TX_RX;
  //LREG (uint16_t, B0_RPCR) = RPCR_ANEG | LEDA_10M_100M | LEDB_TX_RX;
#endif
  /* Turn off the isolation mode, start Auto_Negotiation process. */
  write_PHY (0, 0x3000);

#if !defined (_10MBIT_) && !defined (_100MBIT_) && 0 // !!!! SKIP this
  /* Wait to complete Auto_Negotiation. */
  for (i = 0; i < 150; i++) {
    delay_1ms (100); // replace with mbed waitms() or this is not necessary for Models ????
    stat = read_PHY (1);
    if (stat & 0x0020) {
      /* ANEG_ACK set, autonegotiation finished. */
      break;
    }
  }

  /* Check for the output status of the autoneg. */
  for (i = 0; i < 30; i++) {
    delay_1ms (100); // replace with mbed waitms() or this is not necessary for Models ????
    stat = read_PHY (18);
    if (!(stat & 0x4000)) {
      break;
    }
  }
#else
  stat = read_PHY (18);
#endif

  /* Set the Control Register */
  LREG (uint16_t, BSR)    = 1;
  LREG (uint16_t, B1_CTR) = CTR_LE_ENABLE | CTR_CR_ENABLE | CTR_TE_ENABLE |
                       CTR_AUTO_REL  | CTR_DEFAULT;

  /* Set Receive Control Register, accept Multicast. */
  //LREG (uint16_t, BSR)    = 0;
  //LREG (uint16_t, B0_RCR) = RCR_RXEN | RCR_STRIP_CRC | RCR_ALMUL;

  /* Setup Transmit Control Register. */
  tcr = TCR_TXENA | TCR_PAD_EN;
  if (stat & 0x0040) {
    tcr |= TCR_FDUPLX;
  }
  LREG (uint16_t, B0_TCR) = (uint16_t)tcr;

  /* Reset MMU */
  LREG (uint16_t, BSR)    = 2;
  LREG (uint16_t, B2_MMUCR) = MMU_RESET;
  while (LREG (uint16_t, B2_MMUCR) & MMUCR_BUSY);

  /* Configure the Interrupts, allow  RX_OVRN and RCV intr. */
  LREG (uint8_t,  B2_MSK) = MSK_RX_OVRN | MSK_RCV;

  /* Set all buffers or data in handler for data transmit/receive process. */
  LAN91_SetHandler();
}


void LAN91_SetCallback(lan91_callback_t callback, void *userData)
{
    /* Set callback and userData. */
    lan91c111_handle->callback = callback;
    lan91c111_handle->userData = userData;
}



/*--------------------------- send_buffer Frame ------------------------------------*/

bool LAN91_send_frame (uint32_t *buff, uint32_t *size ) {
  /* Send frame to Lan91C111 ethernet controller */
  uint16_t *dp;
  uint8_t packnr;
  int i;

  /* Select BANK2 */
  LREG (uint16_t, BSR) = 2;

  /* MMU allocate memory for transmitting*/
  LREG (uint16_t, B2_MMUCR) = MMU_ALLOC_TX;
  
  
  i = 300000;
  do {
    if (--i == 0) {
      /* Prevent dead loops. */
      break;
    }
  } while (!(LREG (uint16_t, B2_IST) & IST_ALLOC_INT)); /* Check if Interrupt Status Register been set for MMU Allocate Ready */

  /* If MMU allocate memory failed */
  if (i == 0) {
    /* Failed, Reset MMU */
    LREG (uint16_t, B2_MMUCR) = MMU_RESET;
    while (LREG (uint16_t, B2_MMUCR) & MMUCR_BUSY);
    return false;
  }

  /* set TX package number from allocated package number 
     and also set pointer register as Auto Increase ???? write and transmit set ???? */
  packnr = LREG (uint8_t, B2_ARR);
  LREG (uint8_t, B2_PNR) = packnr;
  LREG (uint16_t, B2_PTR) = PTR_AUTO_INCR;

  /* Reserve space for Status Word */
  LREG (uint16_t, B2_DATA0) = 0x0000;
  LREG (uint16_t, B2_DATA0) = (uint16_t)*size + 6; /* Total = Raw Data Size + STATUS WORD + BYTE COUNT + CONTROL BYTE + LAST BYTE */

  /* Copy frame data to Ethernet controller */
  dp = (uint16_t *)buff;
  for (i = *size; i > 1; i -= 2) {
    LREG (uint16_t, B2_DATA0) = *dp++;
  }
  
  /* If data is odd , Add a control word and odd byte. */
  if (i) {
    LREG (uint16_t, B2_DATA0) = (RFC_CRC | RFC_ODD) | (*dp & 0xFF);
  }
  else {
    /* Add control word. */
    LREG (uint16_t, B2_DATA0) = RFC_CRC;
  }

  /* Select BANK0 */
  LREG (uint16_t, BSR)     = 0;
  /* Enable transmitter. */
  LREG (uint16_t, B0_TCR)  = TCR_TXENA | TCR_PAD_EN;

  /* Enqueue the packet. */
  LREG (uint16_t, BSR)      = 2;
  LREG (uint16_t, B2_MMUCR) = MMU_ENQ_TX;
  
  return true;
}


/*--------------------------- receive buffer frame ----------------------------*/

bool LAN91_receive_frame(uint32_t *buff, uint32_t *size )  {
  // /* Ethernet Controller Interrupt function. for Receiving function*/
  
  uint32_t State, RxLen;
  uint32_t val, *dp;

  /* Select BANK 2 */
  LREG (uint16_t, BSR) = 2;


  /* No receive packets queued in Rx FIFO queue */
  State = LREG (uint16_t, B2_FIFO);
  if (State & FIFO_REMPTY) {
    /* Check if empty packet. Continue While loop ????*/
    return false;
  }


  /* Pointer Register set to  RCV + Auto Increase + Read access
  So that Data Register is use RX FIFO*/
  LREG (uint16_t, B2_PTR) = PTR_RCV | PTR_AUTO_INCR | PTR_READ;
  
  /* Read status word and packet length */ 
  val = LREG (uint32_t, B2_DATA);
  State = val & 0xFFFF;
  RxLen = (val >> 16) - 6; /* Raw Data Size = Total - STATUS WORD - BYTE COUNT - CONTROL BYTE - LAST BYTE */
  
  /* Check State word if Odd number of bytes in a frame. */
  if (State & RFS_ODDFRM) {
    RxLen++;
  }
  

  /* Packet too big, ignore it and free MMU and continue*/
  if (RxLen > FVP_ETH_MTU_SIZE) {
    LREG (uint16_t, B2_MMUCR) = MMU_REMV_REL_RX;
    return false;
  }

  /* if 'alloc_mem()' has failed, ignore this packet. */
  if (buff != NULL) {
    /* Make sure that block is dword aligned */
    RxLen = (RxLen + 3) >> 2;
    *size = RxLen << 2;
    dp = buff;
    for (  ; RxLen; RxLen--) {
      *dp++ = LREG (uint32_t, B2_DATA);
    }
  }

  /* MMU free packet. Remove and Relase from TOP of RX */
  LREG (uint16_t, B2_MMUCR) = MMU_REMV_REL_RX;

  return true;
}

/*--------------------------- Handler ----------------------------------*/


void ETHERNET_Handler(void)
{
  uint32_t val;

  /* Select BANK 2 */
  LREG (uint16_t, BSR) = 2;
  LREG (uint8_t,  B2_MSK) = 0;

  /* Check if Interrupt Status Register been set for Received Packages*/
  if ((val = (LREG (uint8_t, B2_IST) & (IST_RX_OVRN | IST_RCV))) != 0) {

    /* If RX Overrun bit is set, Clear the RX overrun bit. */
    if (val & IST_RX_OVRN) {
      LREG (uint8_t, B2_ACK) = ACK_RX_OVRN;
      //continue;
    }
    
    /* Callback function. */
    if (lan91c111_handle->callback)
    {
        lan91c111_handle->callback(LAN91_RxEvent, lan91c111_handle->userData);
    }

  }

}

/*--------------------------- delay_1ms -------------------------------------*/

static void delay_1ms (uint32_t time) {
  /* Implements delays for the driver */
  /* Note: The execution time of this function depends on the CPU clock,    */
  /* type of memory where the code is executed etc. It should be calibrated.*/
  uint32_t dly;

  for (  ; time; time--) {
    /* Wait 1ms - execution time depends on CPU speed. */
    for (dly = 2500; dly; dly--);
  }
}


/*--------------------------- output_MDO ------------------------------------*/

static void output_MDO (int bit_value) {
  /* Output a bit value to the MII PHY management interface. */
  uint32_t val = MGMT_MDOE;

  if (bit_value) {
    val |= MGMT_MDO;
  }
  LREG (uint16_t, B3_MGMT) = (uint16_t)val;
  LREG (uint16_t, B3_MGMT) = (uint16_t)(val | MGMT_MCLK);
  LREG (uint16_t, B3_MGMT) = (uint16_t)val;
}


/*--------------------------- input_MDI -------------------------------------*/

static int input_MDI (void) {
  /* Input a bit value from the MII PHY management interface. */
  int val = 0;

  LREG (uint16_t, B3_MGMT) = 0;
  LREG (uint16_t, B3_MGMT) = MGMT_MCLK;
  if (LREG (uint16_t, B3_MGMT) & MGMT_MDI) {
    val = 1;
  }
  LREG (uint16_t, B3_MGMT) = 0;
  return (val);
}


/*--------------------------- write_PHY -------------------------------------*/

static void write_PHY (uint32_t PhyReg, int Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  int i;

  LREG (uint16_t, BSR)     = 3;
  LREG (uint16_t, B3_MGMT) = MGMT_MDOE | MGMT_MDO;

  /* 32 consecutive ones on MDO to establish sync */
  for (i = 0; i < 32; i++) {
    LREG (uint16_t, B3_MGMT) = MGMT_MDOE | MGMT_MDO;
    LREG (uint16_t, B3_MGMT) = MGMT_MDOE | MGMT_MDO | MGMT_MCLK;
  }
  LREG (uint16_t, B3_MGMT) = MGMT_MDOE;

  /* start code (01) */
  output_MDO (0);
  output_MDO (1);

  /* write command (01) */
  output_MDO (0);
  output_MDO (1);

  /* write PHY address - which is five 0s for 91C111 */
  for (i = 0; i < 5; i++) {
    output_MDO (0);
  }

  /* write the PHY register to write (highest bit first) */
  for (i = 0; i < 5; i++) {
    output_MDO ((PhyReg >> 4) & 0x01);
    PhyReg <<= 1;
  }

  /* turnaround MDO */
  output_MDO (1);
  output_MDO (0);

  /* write the data value (highest bit first) */
  for (i = 0; i < 16; i++) {
    output_MDO ((Value >> 15) & 0x01);
    Value <<= 1;
  }

  /* turnaround MDO is tristated */
  LREG (uint16_t, B3_MGMT) = 0;
  LREG (uint16_t, B3_MGMT) = MGMT_MCLK;
  LREG (uint16_t, B3_MGMT) = 0;
}


/*--------------------------- read_PHY --------------------------------------*/

static uint16_t read_PHY (uint32_t PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  int i, val;

  LREG (uint16_t, BSR)     = 3;
  LREG (uint16_t, B3_MGMT) = MGMT_MDOE | MGMT_MDO;

  /* 32 consecutive ones on MDO to establish sync */
  for (i = 0; i < 32; i++) {
    LREG (uint16_t, B3_MGMT) = MGMT_MDOE | MGMT_MDO;
    LREG (uint16_t, B3_MGMT) = MGMT_MDOE | MGMT_MDO | MGMT_MCLK;
  }
  LREG (uint16_t, B3_MGMT) = MGMT_MDOE;

  /* start code (01) */
  output_MDO (0);
  output_MDO (1);

  /* read command (10) */
  output_MDO (1);
  output_MDO (0);

  /* write PHY address - which is five 0s for 91C111 */
  for (i = 0; i < 5; i++) {
    output_MDO (0);
  }

  /* write the PHY register to read (highest bit first) */
  for (i = 0; i < 5; i++) {
    output_MDO ((PhyReg >> 4) & 0x01);
    PhyReg <<= 1;
  }

  /* turnaround MDO is tristated */
  LREG (uint16_t, B3_MGMT) = 0;
  LREG (uint16_t, B3_MGMT) = MGMT_MCLK;
  LREG (uint16_t, B3_MGMT) = 0;

  /* read the data value */
  val = 0;
  for (i = 0; i < 16; i++) {
    val <<= 1;
    val |= input_MDI ();
  }

  /* turnaround MDO is tristated */
  LREG (uint16_t, B3_MGMT) = 0;
  LREG (uint16_t, B3_MGMT) = MGMT_MCLK;
  LREG (uint16_t, B3_MGMT) = 0;

  return (val);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
