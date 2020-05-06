/********************************************************
** cmd.c
**
** Debug command processing
**
********************************************************/
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

static uint8_t cmd_version( struct cmd *cmd ) {
  // There are no parameters
  command.n = sprintf( command.buffer, "# %s %d.%d.%d\r\n",BRANCH,MAJOR,MINOR,SUBVER);
  return 1;
}

static uint8_t check_command( struct cmd *cmd ) {
  uint8_t validCmd = 0;

  if( cmd->n > 0 ) {
    switch( cmd->buffer[0] & ~( 'A'^'a' ) ) {
    case 'V':  validCmd = cmd_version( cmd );       break;
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
      if( command.n==0 )
        command.inCmd = 0;
    } else if( byte=='\n' ) {
      command.inCmd = check_command( &command );
      if( command.inCmd ) {
        if( buffer ) (*buffer) = command.buffer;
        if( n ) (*n) = command.n;
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


