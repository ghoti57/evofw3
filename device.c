/********************************************************
** device.c
**
** Device ID 
**
********************************************************/
#include <stddef.h>
#include <avr/boot.h>

#include "device.h"

static uint8_t  DevClass;
static uint32_t DevId;

void device_init( uint8_t class ) {
  DevClass = class;
  DevId = (  ( (uint32_t)boot_signature_byte_get(0x15) << 16 )
           + ( (uint32_t)boot_signature_byte_get(0x16) <<  8 )
           + ( (uint32_t)boot_signature_byte_get(0x17) <<  0 )
          ) & 0x3FFFF;
}

void device_get_id( uint8_t *class, uint32_t *id ) {
  if( class ) *class = DevClass;
  if( id    ) *id    = DevId;
}

