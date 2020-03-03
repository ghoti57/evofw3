/**************************************************************************
** tty.c
** 
** Host facing UART 
*/
#include <avr/interrupt.h>

#include "config.h"
#include "trace.h"
#include "tty.h"

#define DEBUG_TX(_v) DEBUG5(_v)
#define DEBUG_RX(_v) DEBUG6(_v)

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

  if( UCSR0A & ( 1<<UDRE0 ) ) {	// TX buffer is empty
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
	  DEBUG_TX(1);
      UDR0 = byte;
      DEBUG_TX(0);
	}
  }
}

static void tty_tx_put(uint8_t byte) {
  ttyTx[ ttyTx_in ] = byte;
  ttyTx_in = ( ttyTx_in+1 ) % TXBUF;
  if( ttyTx_in == ttyTx_out ) {	// BAD things could happen if we hit this
	ttyTx_out = ( ttyTx_out+1 ) % TXBUF;
  }
}

uint8_t tty_put_str( uint8_t *byte, uint8_t nByte ) {
  uint8_t space = ( ( ( ttyTx_out+TXBUF ) - ttyTx_in - 1 ) % TXBUF );
	
  if( space < nByte )
    return 0;	// Didn't send anything

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

#define XOFF ( 'S' & 0x3F )	// ctrl-S
#define XON  ( 'Q' & 0x3F )	// ctrl-Q

static void tty_rx_control(void) {
  static uint8_t newState, state = XOFF;
  uint8_t bytes = 0;
  if( ttyRx_in != ttyRx_out )
    bytes = ( ttyRx_in+RXBUF  - ttyRx_out ) % RXBUF;

  switch( state ) {
  case XOFF:
    if( bytes <  8 ) {
      newState = XON;
	}
    break;

  case XON:
    if( bytes > RXBUF-8 ) {
      newState = XOFF;
	}
    else if( bytes==0 ) {
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
  if( ttyRx_in == ttyRx_out ) {	// BAD things could happen if we hit this
	ttyRx_out = ( ttyRx_out+1 ) % RXBUF;
  }
//  echo=byte;
  tty_rx_control();
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
  if( UCSR0A & ( 1<<RXC0 ) ) {	// RX buffer is full
    sei();	// Mustn't risk blocking RX edge ISR
    uint8_t byte = UDR0;
	tty_rx_put( byte );
  }
}

/**************************************************************************
** TX Control
*/
void tty_start_tx(void) {
  uint8_t sreg = SREG;
  cli();
  
  UCSR0B &= ~( 1<<UDRIE0 );
  UCSR0B |=  ( 1<<TXEN0 );

  SREG = sreg;
}

void tty_stop_tx(void) {
  uint8_t sreg = SREG;
  cli();
  
  UCSR0B &= ~( 1<<TXEN0 );
  UCSR0B &= ~( 1<<UDRIE0 );

  SREG = sreg;
}

/**************************************************************************
** RX ISR
*/

ISR(TTY_RX_VECT) {
  DEBUG_RX(1);
  tty_do_rx();
  DEBUG_RX(0);
}

void tty_start_rx(void) {
  uint8_t sreg = SREG;
  cli();

  // Enable the interrupt while disabled - RX buffer will be empty
  UCSR0B |= ( 1<<RXCIE0 );
  
  // Then enable RX
  UCSR0B |= ( 1<<RXEN0 );
  
  SREG = sreg;
}

void tty_stop_rx(void) {
  uint8_t sreg = SREG;
  cli();
  
  UCSR0B &= ~( 1<<RXEN0 );
  UCSR0B &= ~( 1<<RXCIE0 );

  SREG = sreg;
}

/**************************************************************************
** Initialisation
*/

static void tty_init_uart( uint32_t Fosc, uint32_t bitrate )
{
  uint32_t ubrr_0, actual_0, error_0;
  uint32_t ubrr_1, actual_1, error_1;

  ubrr_0  = bitrate*8;  // Rounding adjustment
  ubrr_0 += Fosc;
  ubrr_0 /= 16;
  ubrr_0 /= bitrate;
  ubrr_0 -= 1;

  actual_0  = Fosc / 16;
  actual_0 /= ubrr_0+1;

  ubrr_1  = bitrate*4;  // Rounding adjustment
  ubrr_1 += Fosc;
  ubrr_1 /= 8;
  ubrr_1 /= bitrate;
  ubrr_1 -= 1;

  actual_1  = Fosc / 8;
  actual_1 /= ubrr_1+1;

  error_0 = ( actual_0 > bitrate ) ? actual_0 - bitrate : bitrate - actual_0;
  error_1 = ( actual_1 > bitrate ) ? actual_1 - bitrate : bitrate - actual_1;
  if( error_0 <= error_1 ) {  // Prefer U2X0=0
    UCSR0A = 0;
    UBRR0 = ubrr_0;
  } else {
    UCSR0A = ( 1 << U2X0 );
    UBRR0 = ubrr_1;
  }

  UCSR0B = ( 0 << RXCIE0 ) | ( 0 << TXCIE0 ) | ( 0 << UDRIE0 )	// Interrupts disabled
         | ( 0 << RXEN0  ) | ( 0 << TXEN0  )                    // RX+TX disabled
         | ( 0 << UCSZ02 );                                     // 8 Bits
  
  UCSR0C  = ( 0 << UMSEL01 ) | ( 0 << UMSEL00 )   // Asynchronous
          | ( 0 << UPM01   ) | ( 0 << UPM00   )   // Parity disable
          | ( 0 << USBS0   )                      // 1 stop bit
          | ( 1 << UCSZ01  ) |  ( 1 << UCSZ00 )   // 8 data bits
		  | ( 0 << UCPOL0  );
}

void tty_init(void ) {
  tty_init_uart( F_CPU, TTY_BAUD_RATE );
  tty_start_tx();
  tty_start_rx();
}

void tty_work(void) {
  tty_do_tx();
}