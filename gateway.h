/********************************************************
** gateway.c
**
** Act as gateway between seiral host and 
** message processing
**
********************************************************/
#ifndef _GATEWAY_H_
#define _GATEWAY_H_

extern void gateway_init( uint32_t myID );
extern void gateway_work( void );

#endif // _GATEWAY_H_
