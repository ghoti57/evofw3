#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "config.h"
#include "bitstream.h"
#include "tty.h"
#include "cc1101.h"
#include "led.h"
#include "transcoder.h"

void main_init(void) {
  // OSCCAL=((uint32_t)OSCCAL * 10368) / 10000;

  wdt_disable();
  led_init();

#if defined(DEBUG_PORT)
  DEBUG_DDR |= ( DEBUG_PIN1 + DEBUG_PIN2 + DEBUG_PIN3 );
#endif

  // Wire up components
  transcoder_init(&tty_write_str, 0);//&driver_send_byte);
  bs_init();
  tty_init(&transcoder_accept_outbound_byte);
  cc_init(0,0);//&driver_accept_bit, &driver_request_bit);

  led_off();
  sei();
}

void main_work(void) {
  transcoder_work();
  tty_work();
  cc_work();
}

#ifdef NEEDS_MAIN
int main(void) {
  main_init();

  while(1) {
    main_work();
  }
}
#endif
