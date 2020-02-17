#ifndef _RINGBUF_H_
#define _RINGBUF_H_

#include <stdint.h>
#include "config.h"

typedef struct ringbuf {
  uint8_t read;
  uint8_t write;
  const uint8_t length;
  uint8_t buffer[2];
} rb_t;

extern void     rb_put(rb_t *rb, uint8_t data);
extern uint8_t  rb_get(rb_t *rb);
extern void     rb_reset(rb_t *rb);
extern uint8_t  rb_empty(rb_t *rb);
extern uint8_t  rb_full(rb_t *rb);
extern uint8_t rb_space(rb_t *rb);
extern uint8_t rb_available(rb_t *rb);

/********************************************************************
** use RINGBUF to declare ring buffer in code file that owns buffer
** e.g.
** RINGBUF( RX,256 );
** RINGBUF( TX, 64 );
** you cannot refer to the ringbuffer outside that file.
**
** to pass the ringbuffer to the functions declared above use &name.rb
** e.g.
** if( !rb_empty( &RX.rb );
**/
#define RINGBUF( NAME, LEN ) static struct NAME##_rb{ rb_t rb; uint8_t buffer_extension[LEN-2]; } NAME = { { 0,0,LEN,{0,0} }, {0}}

#endif
