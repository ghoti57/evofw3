#ifndef _CC1101_H_
#define _CC1101_H_

#include <stdint.h>

extern uint8_t cc_put_octet( uint8_t octet );
extern void cc_tx_trigger(void);

typedef uint8_t (*accept_bit_fn)(uint8_t);
typedef uint8_t (*request_bit_fn)(void);

extern void cc_init(accept_bit_fn a, request_bit_fn r);
extern void cc_work(void);

#endif
