/***************************************************************
** hw_uart.c
**
** Note:
**   This must be a CPP file to allow it to use the Serial class correctly
**
** HW uart interface to radio
*/

#include "config.h"
#if defined(HWUART)

#include <avr/io.h>
#include <avr/interrupt.h>

#if defined(TX_SYNCH)
#include <string.h>
#endif

#include "frame.h"
#include "uart.h"

#include "debug.h"
#define DEBUG_ISR(_v)      DEBUG1(_v)

/***************************************************************************
** RX Processing - HW UART
*/

static void rx_start(void) {
  UCSR1B |=  ( 1<<RXEN1 )   // Enable Receiver
           | ( 1<<RXCIE1 ); // Enable RX complete interrupt
}

//---------------------------------------------------------------------------------

static void rx_stop(void) {
  UCSR1B &=  ~( 1<<RXCIE1 )  // Disable RX complete interrupt
           & ~( 1<<RXEN1 );  // Stop receiver
}

ISR( USART1_RX_vect ) {
  volatile uint8_t status __attribute__((unused));
  uint8_t data;

DEBUG_ISR(1);
  status = UCSR1A;
  data = UDR1;
  frame_rx_byte(data);
DEBUG_ISR(0);
}

#if defined(TX_SYNCH)
/***************************************************************************
** TX Processing - CC1101 Synchronous
*/
#define MARK  1
#define SPACE 0

static struct tx_data {
  uint8_t byte;
  uint8_t bitNo;
  uint8_t done;
} tx;

static void tx_reset(void) {
  memset( &tx, 0, sizeof(tx) );
}

static void tx_start(void) {
  GDO0_PORT |=  GDO0_IN ;	    // Start in MARK
  GDO0_DDR  |=  GDO0_IN;        // Output

  GDO2_DDR  &= ~GDO2_IN;        // Input

  // Rising edge
  EICRA &= ~GDO2_INT_ISCn0 & ~GDO2_INT_ISCn1;
  EICRA |=  GDO2_INT_ISCn0 |  GDO2_INT_ISCn1;

  EIFR   = GDO2_INT_MASK ;    // Acknowledge any previous edges
  EIMSK |= GDO2_INT_MASK ;    // Enable interrupts

  tx_reset();
}

//---------------------------------------------------------------------------------

static void tx_stop(void) {
  EIMSK &= ~(1 << GDO2_INT_MASK);  // Disable interrupts

  GDO0_PORT &= ~GDO0_IN ;	    // Leave in SPACE
  GDO0_DDR  &= ~GDO0_IN;        // Input
}

//---------------------------------------------------------------------------------

static void tx_bit(void) {
  uint8_t bit;

  if( tx.bitNo==0 ) { // START bit
	bit = SPACE;
  } else if( tx.bitNo<9 ) { // data bit
    uint8_t mask = 1 << ( tx.bitNo - 1 );	// Little Endian
    bit = ( tx.byte & mask ) ? MARK : SPACE;
  } else { // STOP bit
    bit = MARK;
  }

  if( bit ) GDO0_PORT |=  GDO0_IN ;
  else      GDO0_PORT &= ~GDO0_IN ;

  if( tx.bitNo==0 )
  	tx.done = frame_tx_byte(&(tx.byte));
  if( !tx.done )
    tx.bitNo = ( tx.bitNo+1 ) % 10;
  else {
  	frame_tx_byte(&(tx.byte));
    tx_stop();
  }
}

ISR(GDO2_INT_VECT) {	// Transmit next bit
  DEBUG_ISR(1);
  tx_bit();
  DEBUG_ISR(0);
}

#else  // TX_SYNCH

/***************************************************************************
** TX Processing - HW UART
*/

static void tx_start(void) {
  UCSR1B |=  ( 1<<TXEN1 )   // Start Transmitter
           | ( 1<<TXCIE1 )  // Enable TX complete interrupt
           | ( 1<<UDRE1 );  // Enable UDR empty interrupt
  UCSR1A |=  ( 1<<TXC1 );   // Make sure TXC clear
}

//---------------------------------------------------------------------------------

static void tx_stop(void) {
  UCSR1B &=  ~( 1<<UDRE1 )   // Disable UDR empty interrupt 
           & ~( 1<<TXEN1 )   // Disable transmitter
  	       & ~( 1<<TXCIE1 ); // Disable TX complete interrupt
}

//---------------------------------------------------------------------------------

ISR( USART1_UDRE_vect ) {	// Transmit next byte
  uint8_t data;
  uint8_t done;

DEBUG_ISR(1);
  done = frame_tx_byte( &data );
  if( !done ) {
    UDR1 = data;
  } else  {
    UCSR1B &= ~( 1<<UDRE1 );	 // Disable UDR empty interrupt 
  }
DEBUG_ISR(0);
}

ISR( USART1_TX_vect ) {
  uint8_t data;

DEBUG_ISR(1);
  frame_tx_byte( &data );
DEBUG_ISR(0);
}

#endif

/***************************************************************************
** External interface
*/

void uart_rx_enable(void) {
  uint8_t sreg = SREG;
  cli();

  tx_stop();
  rx_start();

  SREG = sreg;
}

//---------------------------------------------------------------------------------

void uart_tx_enable(void) {
  uint8_t sreg = SREG;
  cli();

  rx_stop();
  tx_start();

  SREG = sreg;
}

//---------------------------------------------------------------------------------

void uart_disable(void) {
  uint8_t sreg = SREG;
  cli();

  rx_stop();
  tx_stop();
  
  SREG = sreg;
}

//---------------------------------------------------------------------------------

void uart_work(void) {
}

//---------------------------------------------------------------------------------

#define UBRR(_f,_b) ( ( ( ( (_f)/16 ) + ((_b)/2) )/( _b ) ) - 1 )

void uart_init(void) {
  uint8_t sreg = SREG;
  cli();

  UCSR1A = ( 1<<TXC1 )    // Clear TX interrupt
         | ( 0<<U2X1 )    // No Double speed
         | ( 0<<MPCM1 );  // No Multi-processor Command Mode
  	     
  UCSR1B = ( 0<<RXCIE1 )  // Disable RX complete interrupt
         | ( 0<<TXCIE1 )  // Disable TX complete interrupt
         | ( 0<<UDRIE1 )  // disable UDR Empty interrupt
         | ( 0<<RXEN1 )   // Disable RX 
         | ( 0<<TXEN1 )   // Disable TX 
         | ( 0<<UCSZ12 )  // 8 data bits
         | ( 0<<RXB81 )   // No 9th bit
         | ( 0<<TXB81 );  // No 9th bit
  UCSR1C = ( 0<<UMSEL11 ) | ( 0<<UMSEL10 )  // Asynchronous USART
         | ( 0<<UPM11   ) | ( 0<<UPM10   )  // Parity disabled
         | ( 0<<USBS1   )                   // 1 stop bit
         | ( 1<<UCSZ11  ) | ( 1<<UCSZ10  )  // 8 data bits
         | ( 0<<UCPOL1  ); 	                // No synchronous clock
  UCSR1D = ( 0<<CTSEN )     // Disable CTS
         | ( 0<<RTSEN );    // Disable RTS

  UBRR1H = UBRR(F_CPU,RADIO_BAUDRATE) >> 8;
  UBRR1L = UBRR(F_CPU,RADIO_BAUDRATE);
  
  SREG = sreg;
}

#endif // HWUART
