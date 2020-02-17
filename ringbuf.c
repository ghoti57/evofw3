#include <avr/interrupt.h>
#include "ringbuf.h"

uint8_t rb_empty(rb_t *rb) {
  return rb->read==rb->write;
}

uint8_t rb_full(rb_t *rb) {
  uint8_t full;

  uint8_t sreg = SREG;
  cli();

  full = ( (rb->write+1)%rb->length ) == rb->read ; 
  
  SREG = sreg;

  return full;
}

uint8_t rb_space(rb_t *rb) {
  uint8_t space;
  
  uint8_t sreg = SREG;
  cli();

  space = ( rb->read+rb->length - rb->write - 1 ) %  rb->length ; 
  
  SREG = sreg;

  return space;
}

uint8_t rb_available(rb_t *rb) {
  uint8_t filled;
  
  uint8_t sreg = SREG;
  cli();

  filled = ( rb->write+rb->length - rb->read ) %  rb->length ; 
  
  SREG = sreg;

  return filled;
}

void rb_reset(rb_t *rb) {
  rb->read = rb->write = 0;
}

void rb_put(rb_t *rb, uint8_t data) {

  rb->buffer[rb->write] = data;

  uint8_t sreg = SREG;
  cli();

  rb->write = ( rb->write + 1 ) % rb->length;
  if( rb->write== rb->read ) 
    rb->read = ( rb->read + 1 ) % rb->length;

  SREG = sreg;
}

uint8_t rb_get(rb_t *rb) {
  uint8_t ret=0;

  uint8_t sreg = SREG;
  cli();

  if( rb->read != rb->write ) {
    ret = rb->buffer[rb->read];
    rb->read = ( rb->read + 1 ) % rb->length;
  }

  SREG = sreg;
  return ret;
}
