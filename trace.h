/***************************************************************************
** trace.h
*/
#ifndef _TRACE_H_
#define _TRACE_H_
#include <stdint.h>

#define TRC_RADIO   0x01
#define TRACE_BS    0x02
#define TRACE_MSG   0x04

#define TRC_ERROR   0x10
#define TRC_DETAIL  0x20

extern uint8_t trace0;

#define TRACE( _t) ( trace0 & (_t) )

extern void trace_cmd(uint8_t len, uint8_t *param);
#endif // _TRACE_H_