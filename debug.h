#ifndef _DEBUG_H_
#define _DEBUG_H_

#if defined(DEBUG_PORT)

#define DEBUG_MASK   ( DEBUG_PIN1 | DEBUG_PIN2 | DEBUG_PIN3 | DEBUG_PIN4 | DEBUG_PIN5 | DEBUG_PIN6 )
#define DEBUG1_ON    do{ DEBUG_PORT |=  DEBUG_PIN1; } while(0)
#define DEBUG1_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN1; } while(0)
#define DEBUG2_ON    do{ DEBUG_PORT |=  DEBUG_PIN2; } while(0)
#define DEBUG2_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN2; } while(0)
#define DEBUG3_ON    do{ DEBUG_PORT |=  DEBUG_PIN3; } while(0)
#define DEBUG3_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN3; } while(0)
#define DEBUG4_ON    do{ DEBUG_PORT |=  DEBUG_PIN4; } while(0)
#define DEBUG4_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN4; } while(0)
#define DEBUG5_ON    do{ DEBUG_PORT |=  DEBUG_PIN5; } while(0)
#define DEBUG5_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN5; } while(0)
#define DEBUG6_ON    do{ DEBUG_PORT |=  DEBUG_PIN6; } while(0)
#define DEBUG6_OFF   do{ DEBUG_PORT &= ~DEBUG_PIN6; } while(0)

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
#define DEBUG5_ON   do{} while(0)
#define DEBUG5_OFF  do{} while(0)
#define DEBUG6_ON   do{} while(0)
#define DEBUG6_OFF  do{} while(0)

#endif

#define DEBUG1(_v) do{ if(_v) DEBUG1_ON; else DEBUG1_OFF; }while(0)
#define DEBUG2(_v) do{ if(_v) DEBUG2_ON; else DEBUG2_OFF; }while(0)
#define DEBUG3(_v) do{ if(_v) DEBUG3_ON; else DEBUG3_OFF; }while(0)
#define DEBUG4(_v) do{ if(_v) DEBUG4_ON; else DEBUG4_OFF; }while(0)
#define DEBUG5(_v) do{ if(_v) DEBUG5_ON; else DEBUG5_OFF; }while(0)
#define DEBUG6(_v) do{ if(_v) DEBUG6_ON; else DEBUG6_OFF; }while(0)

#endif // _DEBUG_H_