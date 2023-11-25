#include <avr/interrupt.h>
#include "led.h"
#include "config.h"

#ifdef HAS_LED

inline void led_init() {
  LED_DDR |= (1 << LED_PIN);

  led_on();
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
