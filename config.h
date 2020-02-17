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

#if defined(DEBUG_PORT)

#define DEBUG_MASK   ( DEBUG_PIN1 | DEBUG_PIN2 | DEBUG_PIN3 | DEBUG_PIN4 )

#define DEBUG1_ON    do{ DEBUG_PORT |=  DEBUG_PIN1; } while(0)
#define DEBUG1_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN1; } while(0)
#define DEBUG2_ON    do{ DEBUG_PORT |=  DEBUG_PIN2; } while(0)
#define DEBUG2_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN2; } while(0)
#define DEBUG3_ON    do{ DEBUG_PORT |=  DEBUG_PIN3; } while(0)
#define DEBUG3_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN3; } while(0)
#define DEBUG4_ON    do{ DEBUG_PORT |=  DEBUG_PIN4; } while(0)
#define DEBUG4_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN4; } while(0)

#else

#define DEBUG1_ON   do{}while(0)
#define DEBUG1_OFF  do{}while(0)
#define DEBUG2_ON   do{}while(0)
#define DEBUG2_OFF  do{}while(0)
#define DEBUG3_ON   do{}while(0)
#define DEBUG3_OFF  do{}while(0)
#define DEBUG4_ON   do{}while(0)
#define DEBUG4_OFF  do{}while(0)
#define DEBUG_OUT   do{}while(0)

#endif

#define DEBUG1(_v) do{ if(_v) DEBUG1_ON; else DEBUG1_OFF; }while(0)
#define DEBUG2(_v) do{ if(_v) DEBUG2_ON; else DEBUG2_OFF; }while(0)
#define DEBUG3(_v) do{ if(_v) DEBUG3_ON; else DEBUG3_OFF; }while(0)
#define DEBUG4(_v) do{ if(_v) DEBUG4_ON; else DEBUG4_OFF; }while(0)

#endif