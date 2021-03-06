/**************************************************************************
** tty_usb.cpp
**
** Note:
**   This must be a CPP file to allow it to use the Serial class correctly
**
** Host facing USB
*/
#include "Arduino.h"
#include "config.h"

#if defined(TTY_USB)
extern "C" {

#include "tty.h"
#include "HardwareSerial.h"

/**************************************************************************
** TX
*/

static uint8_t ttyTx_in;
static uint8_t ttyTx_out;
static uint8_t ttyTx[TXBUF];

static uint8_t tty_tx_get(void) {
  uint8_t byte = 0x00;

  if( ttyTx_out != ttyTx_in ) {
    byte = ttyTx[ttyTx_out];
    ttyTx_out = ( ttyTx_out+1 ) % TXBUF;
  }

  return byte;
}

static volatile uint8_t rxControl = 0;
static volatile uint8_t echo = 0;
static void tty_do_tx( void ) {
  uint8_t byte;

  if( Serial.availableForWrite() ) { // TX buffer not full
    if( rxControl ) { // RX Flow control takes priority
      byte = rxControl;
      rxControl = 0;
    } else if( echo ) {
      byte = echo;
      echo = 0;
    } else {
      byte = tty_tx_get();
    }

    if( byte != 0x00 ) {
      Serial.write(byte);
    }
  }
}

static void tty_tx_put(uint8_t byte) {
  ttyTx[ ttyTx_in ] = byte;
  ttyTx_in = ( ttyTx_in+1 ) % TXBUF;
  if( ttyTx_in == ttyTx_out ) { // BAD things could happen if we hit this
    ttyTx_out = ( ttyTx_out+1 ) % TXBUF;
  }
}

uint8_t tty_put_str( uint8_t *byte, uint8_t nByte ) {
  uint8_t space = ( ( ( ttyTx_out+TXBUF ) - ttyTx_in - 1 ) % TXBUF );

  if( space < nByte )
    return 0;  // Didn't send anything

  space = nByte;
  while( nByte ) {
    tty_tx_put( *byte );
    byte++;
    nByte--;
  }

  return space;
}

/**************************************************************************
** RX
*/

static uint8_t ttyRx_in;
static uint8_t ttyRx_out;
static uint8_t ttyRx[RXBUF];

#define XOFF ( 'S' & 0x3F ) // ctrl-S
#define XON  ( 'Q' & 0x3F ) // ctrl-Q

static void tty_rx_control(void) {
  static uint8_t state = XOFF;
  uint8_t newState = state;
  uint8_t bytes = ( ttyRx_in+RXBUF  - ttyRx_out ) % RXBUF;

  switch( state ) {
  case XOFF:
    if( bytes <  8 ) {
      newState = XON;
    }
    break;

  case XON:
    if( bytes > RXBUF-8 ) {
      newState = XOFF;
    } else if( bytes==0 ) {
      // Periodically send XON
    }
    break;
  }

  if( state != newState ) {
    state = newState;
    rxControl = state;
  }
}

static void tty_rx_put(uint8_t byte) {
  ttyRx[ ttyRx_in ] = byte;
  ttyRx_in = ( ttyRx_in+1 ) % RXBUF;
  if( ttyRx_in == ttyRx_out ) { // BAD things could happen if we hit this
    ttyRx_out = ( ttyRx_out+1 ) % RXBUF;
  }
}

uint8_t tty_rx_get(void) {
  uint8_t byte = 0x00;

  if( ttyRx_in != ttyRx_out ) {
    byte = ttyRx[ttyRx_out];
    ttyRx_out = ( ttyRx_out+1 ) % RXBUF;
  }

  tty_rx_control();

  return byte;
}

static void tty_do_rx() {
  if( Serial.available() ) { // RX data available
    uint8_t byte = Serial.read();
    tty_rx_put( byte );
  }
}

/**************************************************************************
** Initialisation
*/

void tty_debug(const char *str) { Serial.print(str); }

void tty_init(void ) {
  Serial.begin(115200);
  while( !Serial ){}
}

void tty_work(void) {
  tty_do_tx();
  tty_do_rx();
}

} // extern "C" {
#endif // TTY_USB

