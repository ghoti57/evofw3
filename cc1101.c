#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "config.h"
#include "bitstream.h"
#include "cc1101.h"

#define FRAME_INT_ENTER DEBUG1_ON
#define FRAME_INT_LEAVE DEBUG1_OFF
#define FIFO_INT_ENTER  DEBUG2_ON
#define FIFO_INT_LEAVE  DEBUG2_OFF

#define CC_READ  0x80
#define CC_BURST 0x40

// Read/Write single/burst
#define CC1100_IOCFG2    0x00
#define CC1100_IOCFG1    0x01
#define CC1100_IOCFG0    0x02
#define CC1100_FIFOTHR   0x03
#define CC1100_SYNC1     0x04
#define CC1100_SYNC2     0x05
#define CC1100_PKTLEN    0x06
#define CC1100_PKTCTRL1  0x07
#define CC1100_PKTCTRL0  0x08
#define CC1100_ADDR      0x09
#define CC1100_CHANNR    0x0A
#define CC1100_FSCTRL1   0x0B
#define CC1100_FSCTRL0   0x0C
#define CC1100_FREQ2     0x0D
#define CC1100_FREQ1     0x0E
#define CC1100_FREQ0     0x0F
#define CC1100_MDMCFG4   0x10
#define CC1100_MDMCFG3   0x11
#define CC1100_MDMCFG2   0x12
#define CC1100_MDMCFG1   0x13
#define CC1100_MDMCFG0   0x14
#define CC1100_DEVIATN   0x15
#define CC1100_MCSM2     0x16
#define CC1100_MCSM1     0x17
#define CC1100_MCSM0     0x18
#define CC1100_FOCCFG    0x19
#define CC1100_BSCFG     0x1A
#define CC1100_AGCCTRL2  0x1B
#define CC1100_AGCCTRL1  0x1C
#define CC1100_AGCCTRL0  0x1D
#define CC1100_WOREVT1   0x1E
#define CC1100_WOREVT0   0x1F
#define CC1100_WORCTRL   0x20
#define CC1100_FREND1    0x21
#define CC1100_FREND0    0x22
#define CC1100_FSCAL3    0x23
#define CC1100_FSCAL2    0x24
#define CC1100_FSCAL1    0x25
#define CC1100_FSCAL0    0x26
#define CC1100_RCCTRL1   0x27
#define CC1100_RCCTRL0   0x28
#define CC1100_FSTEST    0x29
#define CC1100_PTEST     0x2A
#define CC1100_AGCTEST   0x2B
#define CC1100_TEST2     0x2C
#define CC1100_TEST1     0x2D
#define CC1100_TEST0     0x2E

// Strobe commands and registers
#define CC1100_SRES      0x30
#define CC1100_SFSTXON   0x31
#define CC1100_SXOFF     0x32
#define CC1100_SCAL      0x33
#define CC1100_SRX       0x34
#define CC1100_STX       0x35
#define CC1100_SIDLE     0x36
#define CC1100_SWORTIME  0x37
#define CC1100_SWOR      0x38
#define CC1100_SPWD      0x39
#define CC1100_SFRX      0x3A
#define CC1100_SFTX      0x3B
#define CC1100_SWORRST   0x3C
#define CC1100_SNOP      0x3D
#define CC1100_PATABLE   0x3E
#define CC1100_FIFO      0x3F


// Burst mode registers
#define CC1100_PARTNUM        ( CC1100_SRES     | CC_BURST )
#define CC1100_VERSION        ( CC1100_SFSTXON  | CC_BURST )
#define CC1100_FREQEST        ( CC1100_SXOFF    | CC_BURST )
#define CC1100_LQI            ( CC1100_SCAL     | CC_BURST )
#define CC1100_RSSI           ( CC1100_SRX      | CC_BURST )
#define CC1100_MARCSTATE      ( CC1100_STX      | CC_BURST )
#define CC1100_WORTIME1       ( CC1100_SIDLE    | CC_BURST )
#define CC1100_WORTIME0       ( CC1100_SWORTIME | CC_BURST )
#define CC1100_PKTSTATUS      ( CC1100_SWOR     | CC_BURST )
#define CC1100_VCO_VC_DAC     ( CC1100_SPWD     | CC_BURST )
#define CC1100_TXBYTES        ( CC1100_SFRX     | CC_BURST )
#define CC1100_RXBYTES        ( CC1100_SFTX     | CC_BURST )
#define CC1100_RCCTRL1_STATUS ( CC1100_SWORRST  | CC_BURST )
#define CC1100_RCCTRL0_STATUS ( CC1100_SNOP     | CC_BURST )
#define CC1100_RXFIFO         ( CC1100_FIFO     | CC_BURST )
#define CC1100_TXFIFO         ( CC1100_FIFO     | CC_BURST )

#define CC1100_RSSI_OFFSET ( 0 /*74*/ /* CC1101 datasheet table 31 */ )

// Chip Status byte
#define CC_CHIP_RDY    0x80
#define CC_STATE_MASK  0x70
#define CC_FIFO_MASK   0x0F

#define CC_STATE( status ) ( status & CC_STATE_MASK )
#define CC_STATE_IDLE         0x00
#define CC_STATE_RX           0x10
#define CC_STATE_TX           0x20
#define CC_STATE_FSTXON       0x30
#define CC_STATE_CALIBRATE    0x40
#define CC_STATE_SETTLING     0x50
#define CC_STATE_RX_OVERFLOW  0x60
#define CC_STATE_TX_UNDERFLOW 0x70


// CC1101 register settings
static const uint8_t PROGMEM CC_REGISTER_VALUES[] = {
#if defined(USE_FIFO)
  0x00, 0x00,  // IOCFG2 - FIFO
  0x01, 0x2E,  // IOCFG1 - 
  0x02, 0x06,  // IOCFG0 - Frame
  0x03, 0x00,  // FIFO_THR
  0x04, 0xFF,  // SYNC1   1111 1111[1] [0]0000 00
  0x05, 0x80,  // SYNC0,
  0x06, 0xFF,  // PKTLEN
  0x07, 0x80,  // PKTCTRL1,
  0x08, 0x02,  // PKTCTRL0
#else
  0x00, 0x0B,  // CCx_IOCFG2 (Serial Clock. Synchronous to the data in synchronous serial mode.)
  0x01, 0x2E,  // CCx_IOCFG1
  0x02, 0x0C,  // CCx_IOCFG0 (Serial Synchronous Data Output. Used for synchronous serial mode.)
  0x07, 0x00,  // CCx_PKTCTRL1
  0x08, 0x12,  // CCx_PKTCTRL0 (Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins)
#endif
  0x0B, 0x06,  // CCx_FSCTRL1
  0x0D, 0x21,  // CCx_FREQ2
  0x0E, 0x65,  // CCx_FREQ1
  0x0F, 0x6C,  // CCx_FREQ0
  0x10, 0x6A,  // CCx_MDMCFG4
  0x11, 0x83,  // CCx_MDMCFG3 (DRATE_M=131 data rate=38,383.4838867Hz)
#if defined(USE_FIFO)
  0x12, 0x16,  // CCx_MDMCFG2 (GFSK  15/16 Sync Word Carrier sense above threshold)
  0x13, 0x22,  // CCx_MDMCFG1 (CHANSPC_E=2 NUM_PREAMBLE=4 FEC_EN=0)
#else
  0x12, 0x10,  // CCx_MDMCFG2 (GFSK No Sync Word / Preamable)
  0x13, 0x22,  // CCx_MDMCFG1 (CHANSPC_E=2 NUM_PREAMBLE=4 FEC_EN=0)
#endif
  0x14, 0xF8,  // CCx_MDMCFG0
  0x15, 0x50,  // CCx_DEVIATN
  0x16, 0x07,  // CCx_MCSM2
  0x17, 0x2F,  // CCx_MCSM1 CCA_MODE unless currently receiving a packet, RXOFF_MODE stay in RX, TX_OFF_MODE RX
  0x18, 0x18,  // CCx_MCSM0 (0x18=11000 FS_AUTOCAL=1 When going from IDLE to RX or TX)
  0x19, 0x16,  // CCx_FOCCFG
  0x1B, 0x43,  // CCx_AGCCTRL2
  0x1C, 0x40,  // CCx_AGCCTRL1
  0x1D, 0x91,  // CCx_AGCCTRL0
  0x23, 0xE9,  // CCx_FSCAL3
  0x24, 0x2A,  // CCx_FSCAL2
  0x25, 0x00,  // CCx_FSCAL1
  0x26, 0x1F,  // CCx_FSCAL0
  0x29, 0x59,  // CCx_FSTEST
  0x2C, 0x81,  // CCx_TEST2
  0x2D, 0x35,  // CCx_TEST1
  0x2E, 0x09,  // CCx_TEST0
  0x3E, 0xC0   // CCx_PATABLE
};

// Radio states
#define RS_RX            0
#define RS_TX            1
#define RS_CHANGE_TO_TX  2
#define RS_CHANGE_TO_RX  3

static accept_bit_fn accept_bit;
static request_bit_fn request_bit;

static volatile uint8_t radio_state;

enum frame_state {
  FRAME_IDLE,
  FRAME_RX,
  FRAME_TX
};
static volatile uint8_t frame_state;

static void spi_set_clock( uint32_t Fosc, uint32_t sckFreq )
{
  // ATMEGA328 data sheet Table23-5 
  // Relationship between SCK and Oscillator Frequency
  static struct spi_sck_options {
    uint8_t spi2x :1;
    uint8_t spr1  :1;
    uint8_t spr0  :1;
  } const sck_options[8] = {
    { 1,0,0 },  // Fosc/2^0 (1) - not supported - defaults to Fosc/2
    { 1,0,0 },  // Fosc/2^1 (2)   max SCK frequency
    { 0,0,0 },  // Fosc/2^2 (4)
    { 1,0,1 },  // Fosc/2^3 (8)
    { 0,0,1 },  // Fosc/2^4 (16)
    { 1,1,0 },  // Fosc/2^5 (32)
    { 0,1,0 },  // Fosc/2^6 (64)
    { 0,1,1 }   // Fosc/2^7 (128) min SCK frequency
  }, *opt;

  uint8_t power=0;
  while( sckFreq < Fosc ) {
    sckFreq <<= 1;
    power++;
  }
  if( power>7 ) power = 7;
  opt = &sck_options[power];

  SPSR &= ~( 1 << SPI2X );
  SPCR &= ~( 1 << SPR1 );
  SPCR &= ~( 1 << SPR0 );

  SPSR |= opt->spi2x << SPI2X ;
  SPCR |= opt->spr1 << SPR1 ;
  SPCR |= opt->spr0 << SPR0 ;
}

static void spi_init(void) {
  SPI_PORT |= ( 1 << SPI_SCLK );

  SPI_DDR |= ( 1 << SPI_MOSI ) | ( 1 << SPI_SCLK ) | ( 1 << SPI_SS );
  SPI_DDR &= ~(1 << SPI_MISO);

  SPCR = 0;
  spi_set_clock( F_CPU,  SPI_CLK_RATE );
  SPCR |= ( 1 << MSTR ) | ( 1 << SPE ) ;
}

static inline uint8_t spi_send(uint8_t data) {
  SPDR = data;
  while (!(SPSR & (1 << SPIF)));
  return SPDR;
}

static inline void spi_deassert(void) {
  SPI_PORT |= (1 << SPI_SS);
}

static inline void spi_assert(void) {
  SPI_PORT &= ~(1 << SPI_SS);
}

static uint8_t spi_strobe(uint8_t b) {
  uint8_t result;
  spi_assert();
  while (SPI_PORT & (1 << SPI_MISO));
  result = spi_send(b);
  while (SPI_PORT & (1 << SPI_MISO));
  spi_deassert();
  return result;
}

static uint8_t cc_read( uint8_t addr ) {
  uint8_t data ;

  spi_assert();

  while( SPI_PORT & ( 1 << SPI_MISO ) );

  spi_send( addr | CC_READ );
  data = spi_send( 0 );

  spi_deassert();

  return data;
}

static uint8_t cc_write(uint8_t addr, uint8_t b) {
  uint8_t result;

  spi_assert();

  while (SPI_PORT & (1 << SPI_MISO));

  spi_send(addr);
  result = spi_send(b);

  spi_deassert();
  return result;
}

/******************************************************
* It is important to read the fifo with the minimal
* number of SPI cycles because data is accumulating
* while we are reading it.
*
* This is especially the case whenwe process the bytes
* that contain the Evo payload length becaue we don't
* want the last byte of the packet to get into the FIFO
* before we've set the pktLen.
*/
static uint8_t cc_read_fifo(uint8_t *buffer, uint8_t readAll)
{
  uint16_t nByte,nByte1=255;

  while( 1 ) {
    uint8_t status;

    spi_assert();
    while( SPI_PORT & (1 << SPI_MISO) );

    status = spi_send(CC1100_FIFO|0x40|0x80); // FFIFO+read+burst
    nByte = status & 0x0F;
    if( nByte==nByte1   ) break;  // Same 
    if( nByte==nByte1+1 ) break;  // or one byte added to fifo
    nByte1 = nByte;

    while (SPI_PORT & (1 << SPI_MISO));
    spi_deassert();
  }

  if( nByte>0 )
  {
    if( !readAll ) nByte -= 1;
    for( nByte1=0; nByte1<nByte; nByte1++ )
    {
      *buffer = spi_send(0);
      buffer++;
    }
  }

  while (SPI_PORT & (1 << SPI_MISO));
  spi_deassert();

  return nByte;  
}


#if defined(USE_FIFO)

#define FRAME_INT ( 1 << GDO0_INT )
#define FIFO_INT  ( 1 << GDO2_CLK_INT )
#define INT_MASK  ( FRAME_INT | FIFO_INT )

static uint16_t frameLen;
static uint16_t rxBytes;
static uint8_t writePktLen = 1;

static uint8_t rx_data[64];

static void read_fifo(uint8_t readAll)
{
  uint8_t *data =rx_data;
  uint8_t nByte = cc_read_fifo( data, readAll );

  while( nByte-- )
  {
    uint16_t bs_status = bs_accept_octet( *data );
    data++;
    rxBytes++;

    if( bs_status == BS_END_OF_PACKET ) { 
      readAll = 1;
    } else if( bs_status > BS_MAX_STATUS ) { // Final packet length
      if( frameLen==0xFFFF ) {
        frameLen = bs_status;
        cc_write( 0x06, frameLen & 0xFF );
      }
    } else if( bs_status > BS_END_OF_PACKET ) { //  This is an error condition
      if( writePktLen ) {
		// Tell the packet handler to end the frame
	    // but capture a few more bytes for diagnostics
        cc_write( 0x06, ( rxBytes+4 ) & 0xFF );
        cc_write( 0x08, 0x00 );
        writePktLen = 0;
      }
    }

    if( frameLen != 0xFFFF ) {
      while( rxBytes>255 ) {
        rxBytes -= 256;
        frameLen -= 256;
      }

      if( writePktLen &&  frameLen < 256 ) {
        writePktLen = 0; // Only write this once
        cc_write( 0x08, 0x00 );
      }
    }
  }
}

//----------------------------------------------
static void cc_enable_rx(void);
static void cc_start_rx(void);
static void cc_process_rx(void);
static void cc_end_rx(void);

static void cc_enable_tx(void);
static void cc_start_tx(void);
static void cc_process_tx(uint8_t writeAll);
static void cc_end_tx(void);

static uint8_t tx_pending = 0;

//----------------------------------------------
static void cc_enable_rx(void) {
  // Radio automatically switches back to RX after TX frame
  // or stays there after RX frame
  
  cc_write( CC1100_PKTCTRL0, 0x02 ); // Infinite packet mode
}

// Called From Frame interrupt when RX start detected
static void cc_start_rx(void) {
  uint8_t rssi;

  // Configure FIFO interrupt to use RX fifo
  // Signal asserts when FIFO is at or above threshhold
  // Signal clears when FIFO drained below threshhold
  cc_write( CC1100_IOCFG2, 0 );
  cc_write( CC1100_FIFOTHR, 0 );  // 4 bytes in RX FIFO

  EICRA |= ( 1 << GDO2_CLK_INT_ISCn1 );   // Set edge trigger
  EICRA |= ( 1 << GDO2_CLK_INT_ISCn0 );   // ... rising edge

  frameLen = 0xFFFF;
  writePktLen = 1;
  rxBytes = 0;

  // CC1101 has just detected SYNC WORD
  // Tell bitstream new frame has started
  bs_accept_octet(0x00);

  // RSSI value is latched after SYNC WORD
  rssi = cc_read( CC1100_RSSI );
  bs_rx_rssi( rssi-CC1100_RSSI_OFFSET );
}

static void cc_process_rx(void) {
  read_fifo(0);      // Leave at least 1 byte in FIFO (see errata)
  cc_process_tx(0);  // pull pending TX data
}

static void cc_end_rx(void) {
  read_fifo(1);  // We can empty FIFO now
  bs_accept_octet(0xFF);

  if( tx_pending )
    cc_enable_tx();
  else
    cc_enable_rx();
}

//----------------------------------------------

static void cc_enable_tx(void) {
  cc_write( CC1100_PKTCTRL0, 0x02 ); // Infinite packet mode

  if( bs_enable_tx() ) {
    // Kick the radio. If it can detect RX activity nothing will happen
    // Don't do anything else until we actually see a TX frame
    spi_strobe( CC1100_STX );
  }
}

static void cc_start_tx(void) {
  // TX Frame detected
  uint16_t pktLen = bs_start_tx();

  // Configure FIFO interrupt to use TX fifo
  // Signal asserts when FIFO is at or above threshhold
  // Signal clears when FIFO drained below threshhold
  cc_write( CC1100_IOCFG2, 2 );
  cc_write( CC1100_FIFOTHR, 7 );    // 32 bytes in TX FIFO

  EICRA |=  ( 1 << GDO2_CLK_INT_ISCn1 );  // Set edge trigger
  EICRA &= ~( 1 << GDO2_CLK_INT_ISCn0 );  // ... falling edge

  cc_write( CC1100_PKTLEN, pktLen & 0xFF );

  // If all data is in FIFO we can also set fixed packet length
  if( bs_process_tx( 0 ) )
    cc_write( CC1100_PKTCTRL0, 0x00 );

  tx_pending = 0;
}

// Called whenever we have opportunity to update CC1101
static void cc_process_tx( uint8_t writeAll ) {
  // use txSpace to influence how much work we allow BS to do
  uint8_t txSpace = 64 - cc_read( CC1100_TXBYTES );
  if( !writeAll && txSpace > 8 )
    txSpace = 8;

  if( bs_process_tx( txSpace ) ) {
    // Everything is in FIFO 
    if( frame_state == FRAME_TX ) {
      // We never fill the FIFO so we're OK to set fixed packet length
      cc_write( CC1100_PKTCTRL0, 0x00 );
    }
  }
}

static void cc_end_tx(void) {
  cc_enable_rx();

  bs_end_tx();
}

/****************************************************************
* Frame Itterrupt
*/

ISR(GDO0_INTVECT) {
FRAME_INT_ENTER
  // Frame interrupt
  uint8_t status = spi_strobe(CC1100_SNOP);

  switch( frame_state )
  {
  case FRAME_IDLE:
    switch( CC_STATE(status) ) {
      case CC_STATE_RX:
        cc_start_rx();
        EICRA &= ~( 1 << GDO0_INT_ISCn0 );   // Trigger on next falling edge
        frame_state = FRAME_RX;
        break;

      case CC_STATE_TX:
        cc_start_tx();
        EICRA &= ~( 1 << GDO0_INT_ISCn0 );   // Trigger on next falling edge
        frame_state = FRAME_TX;
        break;
    }
    break;

  case FRAME_RX:  // End of RX Frame
    cc_end_rx();
    EICRA |= (1 << GDO0_INT_ISCn0);          // Trigger on next rising edge
    frame_state = FRAME_IDLE;
    break;

  case FRAME_TX:  // End of TX frame;
    cc_end_tx();
    EICRA |= (1 << GDO0_INT_ISCn0);          // Trigger on next rising edge
    frame_state = FRAME_IDLE;
    break;
  }
FRAME_INT_LEAVE
}

static void cc_frame_init(void) {
  EICRA |= (1 << GDO0_INT_ISCn1);          // Set edge trigger
  EICRA |= (1 << GDO0_INT_ISCn0);          // ... rising edge
}

/****************************************************************
* FIFO Itterrupt
*/

ISR(GDO2_CLK_INTVECT) {
  // Fifo Interrupt
FIFO_INT_ENTER
  switch( frame_state ) {
  case FRAME_RX:    cc_process_rx();   break;
  case FRAME_TX:    cc_process_tx(1);  break;
  }
FIFO_INT_LEAVE
}

/**************************************************
* TX Software interrupt
*
* All the information about the data to be transmitted 
* is held in bitstream
*
* We must only talk to the CC1101 from within an ISR
* This avoids conflicts between SPI cycles from 
* RX and TX activity that is simultaneous without
* explicitly having to disable/enable interrupts
*
* Whenever bitstream has updated its status it 
* must call cc_tx_trigger
*/

ISR(SW_INT_VECT) {
  switch( frame_state ) {
    case FRAME_IDLE:
      cc_process_tx( 1 );
      cc_enable_tx();
      break;
    case FRAME_TX:
      cc_process_tx( 1 );
      break;
    case FRAME_RX:
      // Rely on RX activity to process data
      break;
  }
}

static void cc_tx_init(void) {
  SW_INT_DDR  |= SW_INT_PIN;
  SW_INT_MASK |= SW_INT_PIN;
  PCICR |= SW_INT;
}

void cc_tx_trigger(void) {
  SW_INT_PORT |= SW_INT_PIN;
}

uint8_t cc_put_octet( uint8_t octet ) { // Transfer to FIFO
  cc_write( CC1100_FIFO, octet );

  if( frame_state != FRAME_TX )
    tx_pending = 1;

  return 1;
}

#else

static void receive_bit(void) {
  uint8_t bit = (GDO0_DATA_IN >> GDO0_DATA_PIN) & 1;
  if (accept_bit(bit) != 0) {
    // Non-0 is a request to switch to transmit mode. This is called from interrupt
    // handlers. Actual switch is delayed until main loop.
    radio_state = RS_CHANGE_TO_TX;
  }
}

static void send_bit(void) {
  uint8_t bit = request_bit();

  // If something other than 0 or 1 is returned, switch to receive mode. The radio
  // will clock in whatever was set on the data pin last, and will transmit noise,
  // but that doesn't matter.
  if (bit == 0) {
    GDO0_DATA_PORT &= ~(1 << GDO0_DATA_PIN);
  } else if (bit == 1) {
    GDO0_DATA_PORT |= (1 << GDO0_DATA_PIN);
  } else {
    // This is called from interrupt handlers. Actual switch is delayed until main loop.
    radio_state = RS_CHANGE_TO_RX;
  }
}

#define INT_MASK (1 << GDO2_CLK_INT)

ISR(GDO2_CLK_INTVECT) {
  if (radio_state == RS_RX) {
    receive_bit();
  } else if (radio_state == RS_TX) {
    send_bit();
  }
}

#endif

static void cc_enter_rx_mode(void) {
  EIMSK &= ~INT_MASK;            // Disable interrupts

  while ( CC_STATE( spi_strobe( CC1100_SIDLE ) ) != CC_STATE_IDLE );
  spi_strobe( CC1100_SFRX );
  while ( CC_STATE( spi_strobe( CC1100_SRX ) ) != CC_STATE_RX );

#if defined(USE_FIFO)
  frame_state = FRAME_IDLE;

  cc_frame_init();  // Initialise Frame interrupt
  cc_tx_init();     // Initialise Softwre TX  interrupt

  cc_start_rx();

  EIFR  |= INT_MASK;          // Acknowledge any  previous edges
#else
  radio_state = RS_RX;

  GDO0_DATA_DDR &= ~(1 << GDO0_DATA_PIN);    // Set data pin for input
  EICRA |= (1 << GDO2_CLK_INT_ISCn1);      // Set edge trigger
  EICRA |= (1 << GDO2_CLK_INT_ISCn0);      // ... rising edge
#endif
  EIMSK |= INT_MASK;            // Enable interrupts
}

#if !defined(USE_FIFO)
static void cc_enter_tx_mode(void) {
  EIMSK &= ~INT_MASK;            // Disable interrupts

  cc_write( 0x08, 0x02 ); // Set infinite packet

  while ((spi_strobe(CC1100_SIDLE) & CC1100_STATUS_STATE_BM) != CC1100_STATE_IDLE);
  while ((spi_strobe(CC1100_STX) & CC1100_STATUS_STATE_BM) != CC1100_STATE_TX);

#if!defined(USE_FIFO)
  EICRA |= (1 << GDO0_INT_ISCn1);          // Set edge trigger
  EICRA |= (1 << GDO0_INT_ISCn0);          // ... rising edge

  EICRA |= (1 << GDO2_CLK_INT_ISCn1);      // Set edge trigger
  EICRA |= (1 << GDO2_CLK_INT_ISCn0);      // ... rising edge
#else
  radio_state = RS_TX;

  GDO0_DATA_DDR |= (1 << GDO0_DATA_PIN);    // Set data pin for output
  EICRA |= (1 << GDO2_CLK_INT_ISCn1);       // Set rising edge
  EICRA |= (1 << GDO2_CLK_INT_ISCn0);       //   ...
#endif
  EIMSK |= INT_MASK;            // Enable interrupts
}
#endif

void cc_init(accept_bit_fn a, request_bit_fn r) {
  accept_bit = a;
  request_bit = r;

  spi_init();

  spi_deassert();
  _delay_us(1);

  spi_assert();
  _delay_us(10);

  spi_deassert();
  _delay_us(41);

  spi_strobe(CC1100_SRES);
  spi_strobe(CC1100_SCAL);

  for (uint8_t i = 0; i < sizeof(CC_REGISTER_VALUES); ) {
    uint8_t reg = pgm_read_byte(&CC_REGISTER_VALUES[i++]);
    uint8_t val = pgm_read_byte(&CC_REGISTER_VALUES[i++]);
    cc_write(reg, val);
  }

  cc_enter_rx_mode();
}

void cc_work(void) {
#if !defined(USE_FIFO)
  if (radio_state == RS_CHANGE_TO_RX) {
    cc_enter_rx_mode();
  } else if (radio_state == RS_CHANGE_TO_TX) {
    cc_enter_tx_mode();
  }
#endif
}

