#ifndef _CONFIG_H_
#define _CONFIG_H_

#if defined ARDUINO_AVR_PRO
  #include "hw\atm328_pins.h"
  #define GDO0 INT_1
  #define GDO2 INT_0
#elif defined ARDUINO_AVR_NANO
  #include "hw\atm328_pins.h"
  #define GDO0 INT_0
  #define GDO2 INT_1
#else
  #error "CC1101 connection not defined for target"
#endif

#include "debug.h"

#define SPI_CLK_RATE    250000
#define TTY_BAUD_RATE   115200

#endif