/********************************************************
** cc1101_tune.c
**
** Implement process of tuning cc1101 FREQ value
**
********************************************************/
#include <stdio.h>
#include <Arduino.h>

#include "cc1101.h"
#include "cc1101_const.h"
#include "message.h"

#include "cc1101_tune.h"

static uint8_t tuneEnabled = 0;
uint8_t cc_tuneEnabled(void) {
  return tuneEnabled;
}

static enum cc_tune_state {
  TUNE_IDLE,
  TUNE_BEGIN,
  TUNE_WAIT,
  TUNE_CHOP,
  TUNE_ABORT,
  TUNE_STOP  
} tuneState = TUNE_IDLE;
static uint32_t f0;

// Positive searches are typically much quicker than timeouts
// So look for an initial binary chop window by stepping out
// from the centre in relatively small steps until a timeout
// The boundary we're looking for is then in the last step.
#define STEP0 0x10

// This function is only called while tuneEnabled!=0
static uint32_t cc_tune( uint8_t validMsg, uint8_t timeout ) {
  // Store the search control values in 16 bits
  // to avoid excessive 32 bit arithmetic
  static int16_t x,y;
  static int16_t step;
  static int16_t low,high;

  static uint32_t f;
  
  switch( tuneState ) {
  case TUNE_ABORT:
	f = f0;	// Reset to start frequency after abort
	tuneState = TUNE_STOP;
    break;
	
  case TUNE_STOP:
	tuneState = TUNE_IDLE;
    tuneEnabled = 0;
    break;

  case TUNE_IDLE:
	step = -STEP0;	// Initial search is for low limit
	f = f0;
    tuneState = TUNE_BEGIN;
	break;
	
  case TUNE_BEGIN:	// Begin initial search for extreme limit in direction of step
	y = 0;
	x = step;
	f = f0+x;
	tuneState = TUNE_WAIT;
	break;
	
  case TUNE_WAIT:
    if( validMsg ) {
	  // Move the initial search window out a step
      x += step;
	  y += step;
      f += step;
    } else if( timeout ) { 
	  f -= step/2;
	  tuneState = TUNE_CHOP;
    }
    break;
	
  case TUNE_CHOP:
	// Update appropriate boundary
    if( validMsg ) {
      y  = ( x+y )/2;
      f += ( x-y )/2;
	} else if( timeout ) {
      x  = ( x+y )/2;
      f -= ( x-y )/2;
    }

    if( abs(x-y) <= 1  ) {
      if( step<0 ) {
        // Low limit found
        low = y;
        f = f0;
        step = -step; // Search in opposite direction
        tuneState = TUNE_BEGIN;
      } else {
        // High limit found
        high = y;
        f = f0 + ( low+high ) / 2;
        tuneState = TUNE_STOP;
      }
	}
  }
  
  return f;
}

void cc_tune_enable( uint8_t enable ) {
  if( tuneEnabled != enable ) {
    if( enable ) { // Start tune process
	  // Grab the current frequency in case we abort
	  uint8_t startFreq[1+3] ={ CC1100_FREQ2, 0,0,0 };
	  cc_param_read( startFreq[0], sizeof(startFreq)-1, startFreq+1 );

      f0 = ( (uint32_t)startFreq[1] << 16 )
         | ( (uint32_t)startFreq[2] <<  8 )
         | ( (uint32_t)startFreq[3] <<  0 );

      tuneEnabled = 1;
	} else { // Abort tune process
	  tuneState = TUNE_ABORT;
	}
  }
}

#define TUNE_TIMEOUT ( 4UL * 60 * 1000 )

uint8_t cc_tune_work( struct message *msg, char *cmdBuff ) {
  static uint32_t lastF = 0;
  static unsigned long lastValid = 0;
  
  uint8_t nCmd = 0;

  uint32_t F = lastF;
  unsigned long now = millis();

  uint8_t timeout=0;
  uint8_t isValid = msg_isValid(msg);
  if( !isValid ) {
    unsigned long interval = now - lastValid;
	if( interval > TUNE_TIMEOUT )
	  timeout = 1;
  }

  if( isValid || timeout )
    lastValid = now;

  F = cc_tune( isValid, timeout );
  if( lastF != F ) {
	// Build !C command
	nCmd  = sprintf_P( cmdBuff     , PSTR("!c") );
	nCmd += sprintf_P( cmdBuff+nCmd, PSTR(" %02X"), CC1100_FREQ2 );
	nCmd += sprintf_P( cmdBuff+nCmd, PSTR(" %02X"), (uint8_t)( ( F>>16 ) & 0xFF ) );
	nCmd += sprintf_P( cmdBuff+nCmd, PSTR(" %02X"), (uint8_t)( ( F>> 8 ) & 0xFF ) );
	nCmd += sprintf_P( cmdBuff+nCmd, PSTR(" %02X"), (uint8_t)( ( F>> 0 ) & 0xFF ) );
	nCmd += sprintf_P( cmdBuff+nCmd, PSTR("\r\n") );

	lastF = F;

	lastValid = now;
  }
  
  return nCmd;
}