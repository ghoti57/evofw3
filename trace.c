/***************************************************************************
** trace.c
*/
#include "tty.h"
#include "trace.h"


uint8_t get_hex( uint8_t len, uint8_t *param ) {
  uint8_t value = 0;

  if( len > 2 ) len = 2;
  while( len ) {
    uint8_t p = *(param++);
    len--;
    value <<= 4;
    value += ( p>='0' && p<='9' ) ? p - '0'
            :( p>='A' && p<='F' ) ? p - 'A' + 10
            :( p>='a' && p<='f' ) ? p - 'a' + 10
            : 0;
  }

  return value;
}

uint8_t trace0 = TRC_ERROR;//|TRC_RADIO|TRACE_MSG|TRC_DETAIL;
void trace_cmd(uint8_t len, uint8_t *param) {
  if( len > 0 ) trace0 = get_hex(len,param);
  tty_write_str("trace0=");tty_write_hex(trace0);
}
