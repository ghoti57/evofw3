#include <avr/interrupt.h>
#include "led.h"
#include "config.h"

#ifdef HAS_LED

ISR(TIMER1_COMPA_vect) {
  led_toggle();
}

inline void led_init() {
  LED_DDR |= (1 << LED_PIN);

  led_on();

  // One second time to blink LED
  OCR1A = (F_CPU / 1024) - 1;
  TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS12);
  TIMSK1 = (1 << OCIE1A);
}

inline void led_on() {
  LED_PORT |= (1 << LED_PIN);
}

inline void led_off() {
  LED_PORT &= ~(1 << LED_PIN);
}

inline void led_toggle() {
  LED_PORT ^= (1 << LED_PIN);
}

#else

// no LED support
inline void led_init() {}
inline void led_on() {}
inline void led_off() {}
inline void led_toggle() {}

#endif
