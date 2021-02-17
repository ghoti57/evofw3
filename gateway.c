/********************************************************
** gateway.c
**
** Act as gateway between seiral host and 
** message processing
**
********************************************************/
#include <stddef.h>

#include "tty.h"
#include "cmd.h"
#include "trace.h"

#include "cc1101.h"
#include "frame.h"
#include "message.h"

#include "gateway.h"

#define GWAY_CLASS 18
#define GWAY_ID 730

static uint8_t  MyClass = GWAY_CLASS;
static uint32_t MyId = GWAY_ID;

static uint8_t inCmd;
static char *cmdBuff;
static uint8_t nCmd;

void gateway_init( uint32_t myID ) {
  MyId = myID;

  cc_init();
  frame_init();
  msg_init();

  // Force a version string to be printed
  inCmd = cmd(CMD, NULL,NULL );
  inCmd = cmd('V', NULL,NULL );
  inCmd = cmd('\r', &cmdBuff, &nCmd );
}

void gateway_work( void ) {
  static struct message *rx = NULL;
  static struct message *tx = NULL;
  uint8_t byte; 

  frame_work();
  msg_work();

  // Print RX messages
  if( rx ) {
	static char msgBuff[TXBUF];  
	static uint8_t nRx = 0;

	// Do we still have outstanding text to send?
	if( nRx ) {
	  nRx -= tty_put_str( (uint8_t *)msgBuff, nRx );
    }

	if( !nRx ) {
      nRx = msg_print( rx, msgBuff );
      if( !nRx )
        msg_free( &rx );
	}
  } else if( nCmd ) {
    nCmd -= tty_put_str( (uint8_t *)cmdBuff, nCmd );
    if( !nCmd )
      inCmd = 0;
  } else {
    // If we get a message now we'll start printing it next time
    rx = msg_rx_get();
	if( !msg_isValid(rx) ) {
	  if( !TRACE(TRC_ERROR) && !TRACE(TRC_TXERR) )
        msg_free(&rx);	// Silently dump error messages
	}
  }

  // Process serial data from host
  byte = tty_rx_get();

  // Process commands
  if( byte ) {
    if( !nCmd && ( byte==CMD || inCmd ) ) {
      inCmd = cmd( byte, &cmdBuff, &nCmd );
      byte = '\0'; // byte has been used
    }
  }
  
  // Process message bytes
  if( byte ) {
    if( !tx ) tx = msg_alloc();

    if( tx ) { // TX message
      if( msg_scan( tx, byte ) ) {
		if( msg_isValid( tx ) ) {
          msg_change_addr( tx,0, GWAY_CLASS,GWAY_ID , MyClass,MyId );
          msg_tx_ready( &tx );
        } else if( TRACE(TRC_TXERR) ) {
          msg_rx_ready( &tx );
	    } else {
          msg_free( &tx );
        }
	  }
    }
  }

}
