#include <avr/interrupt.h>
#include <avr/io.h>

#include "config.h"
#include "trace.h"
#include "ringbuf.h"
#include "tty.h"
#include "led.h"

static outbound_byte_fn outbound_byte;

RINGBUF( TX, 128 );
RINGBUF( RX, 128 );

// TX complete. Send next char; disable interrupt if nothing else to do
ISR(TTY_UDRE_VECT) {
  UDR0 = rb_get(&TX.rb);
  if( rb_empty(&TX.rb) ) {
    UCSR0B &= ~(1 << UDRIE0);
  }
}

// RX byte ready
ISR(TTY_RX_VECT) {
  uint8_t data  = UDR0;
  uint8_t flags = UCSR0A;

  if ((flags & ((1 << FE0) | (1 << DOR0))) == 0) {
    rb_put( &RX.rb, data);
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
  rb_reset(&TX.rb);
  rb_reset(&RX.rb);

  tty_init_uart( F_CPU, TTY_BAUD_RATE);

  // Enable USART receiver and transmitter and receive complete interrupt
  UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}

#define CMD_CHAR '!'
static uint8_t doCmd = 0;

static struct command {
  uint8_t cmd;
  void (*func)(uint8_t len, uint8_t *param );
} commands[] = {
  { 'T', trace_cmd },
  {'\0', 0 }
};

static void cmd_byte( uint8_t b ) {
  static uint8_t cmd = '\0';
  static uint8_t len = 0;
  static uint8_t param[8];

  if( cmd=='\0' ) {
    if( b!='\r' && b!='\n' )
      cmd = b;
  } else {
    if( b=='\r' ) {
      struct command *command = commands;

      while( ( command->cmd != cmd ) && ( command->cmd != '\0' ) )
        command++;

      if( command->func )
        ( command->func )( len,param );

      cmd = '\0';
      len = 0;
    } else {
      if( len<sizeof(param) )
        param[len++] = b;
    }
  }

  tty_write_char(b);
  if( b=='\r' )
    tty_write_char('\n');
}

void tty_work(void) {
  if( !rb_empty(&RX.rb) ) {
    led_toggle();
    uint8_t b = rb_get(&RX.rb);
    if( b==CMD_CHAR )
    {
      doCmd = 1;
      tty_write_char(b);
    }
    else if( doCmd )
      cmd_byte(b);
    else
      outbound_byte(b);
  }
}

void tty_write_char(char c) {
  while( !( UCSR0A & ( 1<<UDRE0 ) ) );
  UDR0 = c;
}

void tty_write_str(char const *s) {
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
