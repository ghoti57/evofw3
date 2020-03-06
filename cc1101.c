/*********************************************************
*
* cc1101.c
* ========
*
* Hardware interface to TI CC1101 radio chip
*
*/

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "trace.h"

//#define ENABLE_TX

#include "spi.h"
#include "config.h"
#include "cc1101_const.h"
#include "cc1101.h"

// CC1101 register settings
static const uint8_t PROGMEM CC_REGISTER_VALUES[] = {

  CC1100_IOCFG2, 0x2E, //0x00,  // GDO2- FIFO interrupt
  CC1100_IOCFG1, 0x2E,  // GDO1- not used
  CC1100_IOCFG0, 0x2E,  // GDO0- Frame interrupt
  
  CC1100_PKTLEN,   0x00, //
  CC1100_PKTCTRL1, 0x00, //
  CC1100_PKTCTRL0, 0x32, // 0x02, //

  CC1100_FSCTRL1, 0x0F,  //
  CC1100_FREQ2,   0x21,  //
  CC1100_FREQ1,   0x65,  //
  CC1100_FREQ0,   0x6C,  //
  
  CC1100_MDMCFG4, 0x6A,  //
  CC1100_MDMCFG3, 0x83,  // (DRATE_M=131 data rate=38,383.4838867Hz)
  CC1100_MDMCFG2, 0x10,  // (GFSK  15/16 Sync Word Carrier sense above threshold)
  CC1100_MDMCFG1, 0x02,  // (CHANSPC_E=2 NUM_PREAMBLE=4 FEC_EN=0)
//  CC1100_MDMCFG0, 0xF8,  //
  CC1100_DEVIATN, 0x50,  //

  CC1100_MCSM2,   0x07,  //
  CC1100_MCSM1,   0x30,  // CCA_MODE unless currently receiving a packet, RXOFF_MODE to IDLE , TX_OFF_MODE to IDLE
  CC1100_MCSM0,   0x18,  // (0x18=11000 FS_AUTOCAL=1 When going from IDLE to RX or TX)

  CC1100_FOCCFG,  0x16,  //
  
  CC1100_AGCCTRL2, 0x43, //
  CC1100_AGCCTRL1, 0x40, //
  CC1100_AGCCTRL0, 0x91, //

  CC1100_FSCAL3,  0xE9,  //
  CC1100_FSCAL2,  0x2A,  //
  CC1100_FSCAL1,  0x00,  //
  CC1100_FSCAL0,  0x1F,  //

  CC1100_FSTEST,  0x59,  //
  CC1100_TEST2,   0x81,  //
  CC1100_TEST1,   0x35,  //
  CC1100_TEST0,   0x09,  //
  
  CC1100_PATABLE, 0xC3   //
};

static uint8_t cc_read( uint8_t addr ) {
  uint8_t data ;

  spi_assert();

  while( spi_check_miso() );

  spi_send( addr | CC_READ );
  data = spi_send( 0 );

  spi_deassert();

  return data;
}

static uint8_t cc_write(uint8_t addr, uint8_t b) {
  uint8_t result;

  spi_assert();

  while( spi_check_miso() );

  spi_send(addr);
  result = spi_send(b);

  spi_deassert();
  return result;
}

#define INT_MASK 0
void cc_enter_idle_mode(void) {
  EIMSK &= ~INT_MASK;            // Disable interrupts

  while ( CC_STATE( spi_strobe( CC1100_SIDLE ) ) != CC_STATE_IDLE );

  EIFR  |= INT_MASK;          // Acknowledge any  previous edges
}

void cc_enter_rx_mode(void) {
  EIMSK &= ~INT_MASK;            // Disable interrupts

  while ( CC_STATE( spi_strobe( CC1100_SIDLE ) ) != CC_STATE_IDLE );
  cc_write( CC1100_IOCFG2,   0x0D );
  cc_write( CC1100_PKTCTRL0, 0x32 );
  spi_strobe( CC1100_SFRX );
  while ( CC_STATE( spi_strobe( CC1100_SRX ) ) != CC_STATE_RX );

  EIFR  |= INT_MASK;          // Acknowledge any  previous edges
}

void cc_enter_tx_mode(void) {
  EIMSK &= ~INT_MASK;            // Disable interrupts

  while ( CC_STATE( spi_strobe( CC1100_SIDLE ) ) != CC_STATE_IDLE );
  cc_write( CC1100_IOCFG2,   0x0B );
  cc_write( CC1100_PKTCTRL0, 0x12 );
  spi_strobe( CC1100_SFSTXON );
  while ( CC_STATE( spi_strobe( CC1100_STX ) ) != CC_STATE_TX );

  EIFR  |= INT_MASK;          // Acknowledge any  previous edges
}

uint8_t cc_read_rssi(void) {
  // CC1101 Section 17.3
  int8_t rssi = (int8_t )cc_read( CC1100_RSSI );
  rssi = rssi/2 - 74;  // answer in range -138 to -10 
  
  return (uint8_t)( -rssi ); // returns 10 to 138
}
	
void cc_init(void) {
  spi_init();

  spi_deassert();
  _delay_us(1);

  spi_assert();
  _delay_us(10);

  spi_deassert();
  _delay_us(41);

  spi_strobe(CC1100_SRES);
  //spi_strobe(CC1100_SCAL);

  for (uint8_t i = 0; i < sizeof(CC_REGISTER_VALUES); ) {
    uint8_t reg = pgm_read_byte(&CC_REGISTER_VALUES[i++]);
    uint8_t val = pgm_read_byte(&CC_REGISTER_VALUES[i++]);
    cc_write(reg, val);
  }
  
  cc_enter_idle_mode();
}
