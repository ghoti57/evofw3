#include <avr/interrupt.h>
#include <avr/io.h>

#include "config.h"
#include "ringbuf.h"
#include "tty.h"
#include "led.h"

static outbound_byte_fn outbound_byte;

static rb_t tx_buffer;
static rb_t rx_buffer;

// TX complete. Send next char; disable interrupt if nothing else to do
ISR(TTY_UDRE_VECT) {
  UDR0 = rb_get(&tx_buffer);
  if (tx_buffer.nbytes == 0) {
    UCSR0B &= ~(1 << UDRIE0);
  }
}

// RX byte ready
ISR(TTY_RX_VECT) {
  uint8_t data  = UDR0;
  uint8_t flags = UCSR0A;

  if ((flags & ((1 << FE0) | (1 << DOR0))) == 0) {
    rb_put(&rx_buffer, data);
  }
}

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
  error_0 = ( actual_0 > bitrate ) ? actual_0 - bitrate : bitrate - actual_0;

  ubrr_1  = bitrate*4;  // Rounding adjustment
  ubrr_1 += Fosc;
  ubrr_1 /= 8;
  ubrr_1 /= bitrate;
  ubrr_1 -= 1;

  actual_1  = Fosc / 8;
  actual_1 /= ubrr_1+1;
  error_1 = ( actual_1 > bitrate ) ? actual_1 - bitrate : bitrate - actual_1;

  UCSR0C = ( 1 << UCSZ00 ) | ( 1 << UCSZ01 );  // async, 8N1
  if( error_0 <= error_1 ) {  // Prefer U2X0=0
    UCSR0A &= ~( 1 << U2X0 );
    UBRR0 = ubrr_0;
  } else {
    UCSR0A |=  ( 1 << U2X0 );
    UBRR0 = ubrr_1;
  }
}

void tty_init(outbound_byte_fn o) {
  outbound_byte = o;
  rb_reset(&tx_buffer);
  rb_reset(&rx_buffer);

  tty_init_uart( F_CPU, TTY_BAUD_RATE);

  // Enable USART receiver and transmitter and receive complete interrupt
  UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}

void tty_work(void) {
  if (rx_buffer.nbytes) {
    led_toggle();
    uint8_t b = rb_get(&rx_buffer);
    outbound_byte(b);
  }
}

void tty_write_char(char c) {
  // Silently drop characters if the buffer is full...
  if ((tx_buffer.nbytes < RINGBUF_SIZE - 2) ||
      (tx_buffer.nbytes < RINGBUF_SIZE && (c == '\r' || c == '\n'))) {
    rb_put(&tx_buffer, c);
    UCSR0B |= (1 << UDRIE0);  // Enable transmit interrupt
  }
}

void tty_write_str(char *s) {
  while (*s) tty_write_char(*s++);
}

void tty_write_hex(uint8_t byte) {
  static char const hex[16] = {
    '0','1','2','3','4','5','6','7',
    '8','9','A','B','C','D','E','F'
  };

  tty_write_char( hex[ byte >>  4 ] );
  tty_write_char( hex[ byte & 0xF ] );
}
