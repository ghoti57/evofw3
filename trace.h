/***************************************************************************
** trace.h
*/
#ifndef _TRACE_H_
#define _TRACE_H_
#include <stdint.h>

#define TRC_RAW    0x01
#define TRC_ERROR  0x02
#define TRC_TXERR  0x04

extern uint8_t trace0;

#define TRACE( _t) ( trace0 & (_t) )

#endif // _TRACE_H_