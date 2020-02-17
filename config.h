#ifndef _CONFIG_H_
#define _CONFIG_H_

#if defined TARGET_SCC_V2
#  include "hw/scc_v2.h"
#elif defined SHA_NANO_V3
#  include "hw/sha_nano_v3.h"
#elif defined ARDUINO_AVR_PRO
#  include "hw/avr_pro.h"
#elif defined ARDUINO_AVR_NANO
#  include "hw/avr_nano.h"
#else
#  error "No hardware target defined"
#endif

#define USE_FIFO

#if defined(DEBUG_PORT)
#define DEBUG1_ON    ( DEBUG_PORT |=  DEBUG_PIN1 );
#define DEBUG1_OFF   ( DEBUG_PORT &= ~DEBUG_PIN1 );
#define DEBUG2_ON    ( DEBUG_PORT |=  DEBUG_PIN2 );
#define DEBUG2_OFF   ( DEBUG_PORT &= ~DEBUG_PIN2 );
#define DEBUG3_ON    ( DEBUG_PORT |=  DEBUG_PIN3 );
#define DEBUG3_OFF   ( DEBUG_PORT &= ~DEBUG_PIN3 );
#else
#define DEBUG1_ON
#define DEBUG1_OFF
#define DEBUG2_ON
#define DEBUG2_OFF
#define DEBUG3_ON
#define DEBUG3_OFF
#endif

#endif
