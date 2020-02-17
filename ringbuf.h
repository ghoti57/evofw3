#ifndef _RINGBUF_H_
#define _RINGBUF_H_

#include <stdint.h>
#include "config.h"

#if RINGBUF_SIZE < 256
typedef struct
{
  uint8_t putoff;
  uint8_t getoff;
  uint8_t nbytes;
  char buf[RINGBUF_SIZE];
} volatile rb_t;
#else
typedef struct
{
  uint16_t putoff;
  uint16_t getoff;
  uint16_t nbytes;
  char buf[RINGBUF_SIZE];
} volatile rb_t;
#endif

void rb_put(rb_t *rb, uint8_t data);
uint8_t rb_get(rb_t *rb);
void rb_reset(rb_t *rb);
uint8_t rb_empty(rb_t *rb);
uint8_t rb_full(rb_t *rb);

#endif
