#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/power.h>

#include "config.h"
#include "led.h"
#include "tty.h"
#include "spi.h"

#include "gateway.h"

void main_init(void) {
  // OSCCAL=((uint32_t)OSCCAL * 10368) / 10000;

#if defined(DEBUG_PORT)
  DEBUG_DDR  = DEBUG_MASK;
  DEBUG_PORT = 0;
#endif

  wdt_disable();
  clock_prescale_set(clock_div_1);

  led_init();
  tty_init();
  spi_init();

  gateway_init();

  sei();
}

void main_work(void) {
  gateway_work();
  tty_work();
}

#ifdef NEEDS_MAIN
int main(void) {
  main_init();

  while(1) {
    main_work();
  }
}
#endif
