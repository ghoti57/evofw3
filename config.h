#ifndef _CONFIG_H_
#define _CONFIG_H_

#if defined ARDUINO_EVOFW3_ATMEGA32U4
#include "atm32u4_pins.h"
#elif defined ARDUINO_EVOFW3_ATMEGA328P
#include "atm328_pins.h"

#elif defined ARDUINO_AVR_PRO
  #define SWUART
  #define GDO0 INT0
  #define GDO2 INT1
  #define CCSEL 2
  #include "atm328_pins.h"
#elif defined ARDUINO_AVR_NANO
  #define SWUART
  #define GDO0 INT1
  #define GDO2 INT0
  #define CCSEL 2
  #include "atm328_pins.h"
#elif defined ARDUINO_AVR_LEONARDO
  #define HWUART
  #define CCSEL 6
  #include "atm32u4_pins.h"
#else
  #error "CC1101 connection not defined for target"
#endif

#include "debug.h"

#define SPI_CLK_RATE    250000
#define TTY_BAUD_RATE   115200

#endif
