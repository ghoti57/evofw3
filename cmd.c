/********************************************************
** cmd.c
**
** Debug command processing
**
********************************************************/
#include <avr/pgmspace.h>

#include <string.h>
#include <stdio.h>

#include "tty.h"

#include "version.h"
#include "cmd.h"

static struct cmd {
  char buffer[TXBUF];
  uint8_t n;
  uint8_t inCmd;
} command;

static void reset_command(void) {
  memset( &command, 0 , sizeof(command) );
}

//------------------------------------------------------------------------

static uint8_t get_hex( uint8_t len, char *param ) {
  uint8_t value = 0;

  if( len > 2 ) len = 2;
  while( len ) {
    char p = *(param++);
    len--;
    value <<= 4;
    value += ( p>='0' && p<='9' ) ? p - '0'
            :( p>='A' && p<='F' ) ? p - 'A' + 10
            :( p>='a' && p<='f' ) ? p - 'a' + 10
            : 0;
  }

  return value;
}

uint8_t trace0 = 0;// TRC_RAW;
static uint8_t cmd_trace( struct cmd *cmd ) {

  if( cmd->n > 1 )
    trace0 = get_hex( cmd->n-1, cmd->buffer+1 );

  command.n = sprintf_P( command.buffer, PSTR("# !T=%02x\r\n"),trace0);

  return 1;
}

//------------------------------------------------------------------------

static uint8_t cmd_version( struct cmd *cmd __attribute__((unused))) {
  // There are no parameters
  command.n = sprintf_P( command.buffer, PSTR("# %s %d.%d.%d\r\n"),BRANCH,MAJOR,MINOR,SUBVER);
  return 1;
}

//------------------------------------------------------------------------

static uint8_t check_command( struct cmd *cmd ) {
  uint8_t validCmd = 0;

  if( cmd->n > 0 ) {
    switch( cmd->buffer[0] & ~( 'A'^'a' ) ) {
    case 'V':  validCmd = cmd_version( cmd );       break;
    case 'T':  validCmd = cmd_trace( cmd );         break;
    }
  }

  return validCmd;
}

uint8_t cmd( uint8_t byte, char **buffer, uint8_t *n ) {
  if( byte==CMD ) {
    reset_command();
    command.inCmd = 1;
  } else if( command.inCmd ) {
    if( byte=='\r' ) {
      if( command.n==0 ) {
        command.inCmd = 0;
      } else {
       command.inCmd = check_command( &command );
        if( command.inCmd ) {
          if( buffer ) (*buffer) = command.buffer;
          if( n ) (*n) = command.n;
        }
      }
    } else if ( byte >= ' ' ) { // Printable chararacter
      if( command.n < TXBUF )
        command.buffer[ command.n++] = byte;
      else
        command.inCmd = 0;
    }
  }

  return command.inCmd;
}

