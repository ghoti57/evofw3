#ifndef _CONFIG_H_
#define _CONFIG_H_

#if defined ARDUINO_AVR_PRO
  #define GDO0 INT0
  #define GDO2 INT1
  #include "hw\atm328_pins.h"
#elif defined ARDUINO_AVR_NANO
  #define GDO0 INT1
  #define GDO2 INT0
  #include "hw\atm328_pins.h"
#else
  #error "CC1101 connection not defined for target"
#endif

#include "debug.h"

#define SPI_CLK_RATE    250000
#define TTY_BAUD_RATE   115200

#endif