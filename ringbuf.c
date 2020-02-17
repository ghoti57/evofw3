#include <avr/interrupt.h>
#include "ringbuf.h"

void rb_reset(rb_t *rb) {
  rb->getoff = rb->putoff = rb->nbytes = 0;
}

void rb_put(rb_t *rb, uint8_t data) {
  uint8_t sreg = SREG;
  cli();

  if( rb->nbytes < RINGBUF_SIZE ) {
    rb->nbytes++;
    rb->buf[rb->putoff] = data;
    rb->putoff = ( rb->putoff + 1 ) % RINGBUF_SIZE;
  }

  SREG = sreg;
}

uint8_t rb_get(rb_t *rb) {
  uint8_t ret=0;

  uint8_t sreg = SREG;
  cli();

  if( rb->nbytes > 0 ) {
    rb->nbytes--;
    ret = rb->buf[rb->getoff];
    rb->getoff = ( rb->getoff + 1 ) % RINGBUF_SIZE;
  }

  SREG = sreg;
  return ret;
}

uint8_t rb_empty(rb_t *rb) { return rb->nbytes==0; }
uint8_t rb_full(rb_t *rb) { return rb->nbytes>=RINGBUF_SIZE; }

