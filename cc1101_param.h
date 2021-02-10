/*********************************************************
*
* cc1101_param.h
* Parameter management for cc1101
*/
#ifndef _CC1101_PARAM_H_
#define _CC1101_PARAM_H_

#include "cc1101_const.h"

extern uint8_t cc_cfg_default( uint8_t param, uint8_t nParam );
extern uint8_t cc_cfg_get( uint8_t param, uint8_t *buff, uint8_t nParam );
extern uint8_t cc_cfg_set( uint8_t param, uint8_t *buff, uint8_t nParam );

extern uint8_t cc_pa_default( void );
extern uint8_t cc_pa_get( uint8_t *paTable );

#endif // _CC1101_PARAM_H_