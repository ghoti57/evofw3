#ifndef _CONFIG_H_
#define _CONFIG_H_

#if defined ARDUINO_EVOFW3_ATMEGA32U4
#include "atm32u4_pins.h"
#define HOST_RATE 0

#elif defined ARDUINO_EVOFW3_ATMEGA328P
#include "atm328_pins.h"

#else
#error "CC1101 connection not defined for target - use evofw3_avr board definitions"
#endif

#include "debug.h"

#define SPI_CLK_RATE    250000

#if defined(HOST_RATE)
#define TTY_BAUD_RATE   HOST_RATE
#else
#error No Host baud rate defined
#endif


#endif
