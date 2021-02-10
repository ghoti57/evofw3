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
#include "cc1101_param.h"
#include "cc1101.h"

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

#if defined GDO2_INT_MASK
#define INT_MASK GDO2_INT_MASK
#else
#define INT_MASK 0
#endif

void cc_enter_idle_mode(void) {
  EIMSK &= ~INT_MASK;            // Disable interrupts

  while ( CC_STATE( spi_strobe( CC1100_SIDLE ) ) != CC_STATE_IDLE );

  EIFR  |= INT_MASK;          // Acknowledge any  previous edges
}

void cc_enter_rx_mode(void) {
  EIMSK &= ~INT_MASK;            // Disable interrupts

  while ( CC_STATE( spi_strobe( CC1100_SIDLE ) ) != CC_STATE_IDLE );
  spi_strobe( CC1100_SFRX );
  while ( CC_STATE( spi_strobe( CC1100_SRX ) ) != CC_STATE_RX );

  EIFR  |= INT_MASK;          // Acknowledge any  previous edges
}

void cc_enter_tx_mode(void) {
  EIMSK &= ~INT_MASK;            // Disable interrupts

  while ( CC_STATE( spi_strobe( CC1100_SIDLE ) ) != CC_STATE_IDLE );
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

uint8_t cc_param( uint8_t reg, uint8_t nReg, uint8_t *param ) {
  uint8_t valid = 0;

  if( reg<CC1100_PARAM_MAX && (reg+nReg)<CC1100_PARAM_MAX ) {
    uint8_t eimsk = EIMSK;
    valid = 1;
	
    cc_enter_idle_mode();
    while( nReg && reg<CC1100_PARAM_MAX ) {
      cc_write( reg, (*param) );
	  reg++; nReg--; param++;
    }
    cc_enter_rx_mode();

    EIMSK = eimsk;
  }
  
  return valid;
}

void cc_param_read( uint8_t reg, uint8_t nReg, uint8_t *param ) {
  if( param ) {
    while( nReg && reg<CC1100_PARAM_MAX ) {
      (*param) = cc_read( reg );
	  reg++; nReg--; param++;
    }
  }
}

void cc_init(void) {
  uint8_t param[CC1100_PARAM_MAX];
  uint8_t i,len;
  
  spi_init();

  spi_deassert();
  _delay_us(1);

  spi_assert();
  _delay_us(10);

  spi_deassert();
  _delay_us(41);

  spi_strobe(CC1100_SRES);
  //spi_strobe(CC1100_SCAL);
  
  len = cc_cfg_get( 0, param, sizeof(param) );
  for ( i=0 ; i<len ; i++ )
    cc_write( i, param[i] );

  len = cc_pa_get( param );
  for ( i=0 ; i<len ; i++ )
    cc_write( CC1100_PATABLE, param[i]);
  
  cc_enter_idle_mode();
}
