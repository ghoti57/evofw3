/********************************************************
** device.h
**
** Device ID 
**
********************************************************/
#ifndef _DEVICE_H_
#define _DEVICE_H_

extern void device_init( uint8_t  class );
extern void device_get_id( uint8_t *class, uint32_t *id );

#endif // _DEVICE_H_