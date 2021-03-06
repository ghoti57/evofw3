/****************************************************************************
** nv.h
**
** Non-volatile parameter management
*/
#ifndef _NV_H_
#define _NV_H_

#define _NV_PARAM_LIST \
  _NV_PARAM( NV_CC_PARAM, 'C', CC1100_PARAM_MAX, "CC1101 Parameters" ) \
  _NV_PARAM( NV_CC_PA,    'P', CC1100_PA_MAX,    "CC1101 Power Table" ) \
  
#define _NV_PARAM( _e,_i,_l,_t ) ,_e
enum nv_param_type{ NV_NONE _NV_PARAM_LIST , NV_MAX };
#undef _NV_PARAM

extern void nv_reset(void);

extern uint8_t nv_param_len( uint8_t param );
extern uint8_t nv_param_read( uint8_t param, uint8_t offset, uint8_t *buff, uint8_t buffLen );
extern uint8_t nv_param_write( uint8_t param, uint8_t offset, uint8_t *buff, uint8_t buffLen );

#endif // _NV_H_|