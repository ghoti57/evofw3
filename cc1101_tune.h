/********************************************************
** cc1101_tune.h
**
** Implement process of tuning cc1101 FREQ value
**
********************************************************/
#ifndef _CC1101_TUNE_H_
#define _CC1101_TUNE_H_

#include "message.h"

extern uint8_t cc_tuneEnabled(void);
extern void cc_tune_enable( uint8_t enable );
extern uint8_t cc_tune_work( struct message *msg, char *cmdBuff );

#endif // _CC1101_TUNE_H_