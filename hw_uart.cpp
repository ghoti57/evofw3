/***************************************************************
** hw_uart.cpp
**
** Note:
**   This must be a CPP file to allow it to use the Serial class correctly
**
** HW uart interface to radio
*/

#include "config.h"
#if defined(CC_SERIAL)
#include "arduino.h"

extern "C" {

#include <string.h>
#include <util/delay.h>

#include <avr/interrupt.h>

#include "frame.h"
#include "uart.h"

#define DEBUG_ISR(_v)      DEBUG1(_v)


/***********************************************************************************
** RX Frame state machine
*/

enum uart_rx_states {
  RX_OFF,
  RX_IDLE,    // Make sure we've seen an edge for valid interval calculations
  RX_HIGH,    // Check HIGH signal, includes SYNC0 check (0xFF)
  RX_LOW,     // Check LOW signal
  RX_SYNC1,   // Check for SYNC1 (0x00) - revert to RX_HIGH if not found
  RX_STOP,    // Wait for STOP bit to obtain BYTE SYNCH
  RX_SYNCH,   // Gather bytes and report them to frame layer
  RX_SYNCH0,  // Clock recovery
};


#define MAX_EDGE 24
static struct uart_rx_state {
  uint8_t state;

  uint8_t byte;
} rx;

static void rx_reset(void) {
  memset( &rx, 0, sizeof(rx) );
}

//---------------------------------------------------------------------------------

static void rx_init(void) {
}


//---------------------------------------------------------------------------------

static void rx_start(void) {
}

//---------------------------------------------------------------------------------

static void rx_stop(void) {
  rx.state = RX_OFF;
}


/***************************************************************************
** TX Processing
*/

enum uart_tx_states {
  TX_OFF,
  TX_IDLE,
  TX
};


static struct uart_tx_state {
  uint8_t state;

  uint8_t byte;
  uint8_t done;
  uint8_t writeLen;
} tx;

static void tx_reset(void) {
  memset( &tx, 0, sizeof(tx) );
}

//---------------------------------------------------------------------------------

static void tx_init(void) {
}

//---------------------------------------------------------------------------------

static void tx_start(void) {
  tx.done = 0;
  tx.writeLen = CC_SERIAL.availableForWrite();
}

//---------------------------------------------------------------------------------

static void tx_stop(void) {
  tx.state = TX_OFF;
}


/***************************************************************************
** External interface
*/

void uart_rx_enable(void) {
  uint8_t sreg = SREG;
  cli();

  tx_stop();
  rx_reset();
  rx.state = RX_IDLE;

  SREG = sreg;

  rx_start();
}

void uart_tx_enable(void) {
  uint8_t sreg = SREG;
  cli();

  rx_stop();
  tx_reset();
  tx.state = TX_IDLE;

  SREG = sreg;

  tx_start();
}

void uart_disable(void) {
  uint8_t sreg = SREG;
  cli();
	
  rx_stop();
  tx_stop();

  SREG = sreg;
}

void uart_work(void) {

  if( rx.state != RX_OFF ) {
    if( CC_SERIAL.available() ) {
      rx.byte = CC_SERIAL.read();
  	  frame_rx_byte(rx.byte);
    }
  }

  if( tx.state != TX_OFF ) {
  	if( !tx.done ) {
      if( CC_SERIAL.availableForWrite() ) {
        tx.done = frame_tx_byte(&(tx.byte));
        CC_SERIAL.write(tx.byte);
      }
    } else {
      if( CC_SERIAL.availableForWrite() == tx.writeLen )
        frame_tx_byte(&tx.byte);    
    }
  }
}

void uart_init(void) {
  uint8_t sreg = SREG;
  cli();

  CC_SERIAL.begin(RADIO_BAUDRATE);
  while( !CC_SERIAL ){}
  
  rx_init();
  tx_init();  

  SREG = sreg;
}

} // extern "C"
#endif // !CC_SERIAL
