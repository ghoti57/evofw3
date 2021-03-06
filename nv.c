/****************************************************************************
** nv.c
**
** Non-volatile parameter management
**
** Parameters are stored as TLV
** T identifies the parameter block
** L specifies the number of bytes that follow in
** V the value
*/
#include <avr/eeprom.h>
#include <avr/eeprom.h>

#include "cc1101_const.h"

#include "nv.h"

/***********************************************************************************
* eeprom access
*/
static uint8_t EEMEM eeprom[E2END+1];
#define EE_END &( eeprom[E2END+1] )

static uint8_t *ee_find_param( uint8_t id, uint8_t *len ) {
  uint8_t *ee = eeprom;
  uint8_t t,l=0;
  
  while( ee<EE_END-1 ) {
    t = eeprom_read_byte( ee );
    l = eeprom_read_byte( ee+1 );

    if( t==id ) { // Found it
      if( ee+2+l > EE_END )
        l = EE_END-ee-2;
      break;		
	}
	
    if( t==0xFF ) { // no more data
	  l = 0;
	  ee = EE_END;
	  break;
	}
	
	// Try next
	ee += 2 + l;
  }
  
  if( ee>EE_END-2 )
    ee = EE_END;
	
  if( len ) (*len) = l;
  
  return ee;
}

static uint8_t *ee_find_free( uint8_t *len ) {
  uint8_t *ee = ee_find_param( 0xFF, len );
  return ee;
};

static uint8_t ee_read_param( uint8_t id, uint8_t offset, uint8_t *buff, uint8_t buffLen ) {
  uint8_t len=0;
  uint8_t *ee = ee_find_param( id, &len );

  if( ee<EE_END ) {
    if( offset>len )
      len = 0;
	else if( offset+buffLen > len )
      len = len - offset;
    else
      len = buffLen;
  
    if( len > 0 )
      eeprom_read_block( buff, ee+2+offset, len );
  }

  return len;
}

static uint8_t ee_write_param( uint8_t id, uint8_t paramLen, uint8_t offset, uint8_t *buff, uint8_t buffLen ) {
  uint8_t len;
  uint8_t *ee = ee_find_param( id, &len );

  if( ee==EE_END && paramLen==buffLen ) { // Doesn't exist yet
	ee = ee_find_free( &len );
	if( len>paramLen ) {
	  eeprom_write_byte( ee, id );
	  eeprom_write_byte( ee+1, paramLen );
	  len = paramLen;
	} else {
	  len = 0;
	}
  }
  
  if( ee<EE_END ) {
    if( offset>len )
      len = 0;
    else if( offset+buffLen > len )
      len = len - offset;
    else
      len = buffLen;

    if( len > 0 )
      eeprom_update_block( buff, ee+2+offset, len );
  }

  return len;
}

void nv_reset(void) {
  uint8_t buff[16] = { 0xFF,0xFF,0xFF,0xFF , 0xFF,0xFF,0xFF,0xFF , 0xFF,0xFF,0xFF,0xFF , 0xFF,0xFF,0xFF,0xFF };

  uint8_t *ee;
  for( ee=eeprom; ee<EE_END ; ee+=sizeof(buff) )
    eeprom_update_block( buff, ee, sizeof(buff) );
}

/***********************************************************************************
* NV Parameter API
*/
struct nv_param {
  uint8_t id;
  uint8_t len;
//  char *  desc;
};

static struct nv_param const *nv_param_lookup( uint8_t param ) {
#define _NV_PARAM( _e,_i,_l,_t ) ,{ _i, _l }
static struct nv_param const nv_param_list[NV_MAX] = { { '\0', 0 } _NV_PARAM_LIST };
#undef _NV_PARAM
  struct nv_param const *pParam = NULL;
  
  if( param < NV_MAX )
    pParam = nv_param_list + param;

  return pParam;
}

uint8_t nv_param_len( uint8_t param ) {
  uint8_t len = 0;

  struct nv_param const *pParam = nv_param_lookup( param );
  if( pParam )
    len = pParam->len;

  return len;	
}

uint8_t nv_param_read( uint8_t param, uint8_t offset, uint8_t *buff, uint8_t buffLen ) {
  uint8_t nRead = 0;

  struct nv_param const *pParam = nv_param_lookup( param );
  if( pParam )
    nRead = ee_read_param( pParam->id, offset, buff, buffLen );

  return nRead;
}

uint8_t nv_param_write( uint8_t param, uint8_t offset, uint8_t *buff, uint8_t buffLen ) {
  uint8_t nWrite=0;
  
  struct nv_param const *pParam = nv_param_lookup( param );
  if( pParam )
    nWrite = ee_write_param( pParam->id, pParam->len, offset,buff,buffLen );

  return nWrite;
}
