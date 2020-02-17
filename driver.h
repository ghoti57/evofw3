#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdint.h>

typedef void (*inbound_byte_fn)(uint8_t b, uint8_t end);

void driver_init(inbound_byte_fn in);
void driver_work(void);
uint8_t driver_accept_bit(uint8_t bit);
uint8_t driver_request_bit(void);
void driver_send_byte(uint8_t b, uint8_t end);

#endif
