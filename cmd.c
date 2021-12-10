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
#include "cc1101.h"
#include "cc1101_param.h"
#include "cc1101_tune.h"
#include "nv.h"
#include "device.h"

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

uint8_t trace0 = 0;
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

static uint8_t cmd_id( struct cmd *cmd __attribute__((unused))) {
  uint8_t  class;
  uint32_t id;
  device_get_id( &class, &id );

  // There are no parameters
  command.n = sprintf_P( command.buffer, PSTR("# %02hu:%06lu\r\n"),class,id);
  return 1;
}

//------------------------------------------------------------------------
#if defined(DFU)
static void dfu_start_bootloader(void) {
  UDCON = 1;
  USBCON = (1<<FRZCLK);  // disable USB
  UCSR1B = 0;
  EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
  TIMSK0 = 0; TIMSK1 = 0; TIMSK3 = 0; TIMSK4 = 0; UCSR1B = 0; TWCR = 0;
  DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0; TWCR = 0;
  PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
  /* move interrupt vectors to bootloader section and jump to bootloader */
  MCUCR = _BV(IVCE);
  MCUCR = _BV(IVSEL);
  asm volatile("jmp 0x3800");
}
#endif

static uint8_t cmd_boot(struct cmd *cmd __attribute__((unused))) {
#if defined(DFU)
  dfu_start_bootloader();
#endif
  return 0;
}

//------------------------------------------------------------------------
// !C <offset> <data> [<data> ...]
static uint8_t cmd_cc1101(struct cmd *cmd) {
  uint8_t validCmd = 0;

  if( cmd->n > 1 ) {
    switch( cmd->buffer[1] ) {
    case 'R':
	case 'r':
      cc_cfg_default(0,CC1100_PARAM_MAX);
      cc_pa_default();
	  
      cmd->n = sprintf_P( cmd->buffer, PSTR("# !%c Reset\r\n"), cmd->buffer[0]);
      validCmd = 1;
	  break;

    case 'S':
    case 's': {
	  uint8_t param[CC1100_PARAM_MAX];

      cc_param_read( 0, CC1100_PARAM_MAX, param );
      cc_cfg_set( 0, param, CC1100_PARAM_MAX );
	  
      cmd->n = sprintf_P( cmd->buffer, PSTR("# !%c Saved\r\n"), cmd->buffer[0]);
      validCmd = 1;
      break;
      }

    case ' ': {
      uint8_t nParam=0;
      uint8_t param[CC_MAX_PARAM];
      uint8_t start,end;
      uint8_t n = cmd->n;

      // Skip command characters
      end = start = 2;

      while( n > end ) {
        while( n>start && cmd->buffer[start]==' ' ) start++;	// Skip spaces
        end = start;
        while( n>end && cmd->buffer[end]!=' ' ) end++;	// find end of param

        if( end>start )
          param[nParam++] = get_hex( end-start, cmd->buffer+start );

        start = end;
        if( nParam == CC_MAX_PARAM )
          break;
      }

      if( nParam > 2 ) {
        cmd->n = sprintf_P( cmd->buffer, PSTR("# !%c"), cmd->buffer[0] );
        cmd->n += sprintf_P( cmd->buffer+cmd->n, PSTR(" %02x"), param[0] );
        for( n=1 ; n<nParam ; n++ )
          cmd->n += sprintf_P( cmd->buffer+cmd->n, PSTR(" %02x"), param[n] );
        cmd->n += sprintf_P( cmd->buffer+cmd->n, PSTR("\r\n") );

        if( param[0]<CC1100_PARAM_MAX && param[0]+nParam-2<CC1100_PARAM_MAX ) {
          cc_param( param[0], nParam-1,param+1 );
          validCmd = 1;
        }
      }
	  break;
	  }
    }
  }

  return validCmd;
}

static uint8_t cmd_cc_tune(struct cmd *cmd) {
  uint8_t validCmd = 0;
  uint8_t param[CC_MAX_PARAM];

  if( cmd->n > 1 ) {
    switch( cmd->buffer[1] & ~( 'A'^'a' ) ) {
    case 'T':
      cc_tune_enable( 1 );

      cmd->n = sprintf_P( cmd->buffer, PSTR("# !%c Tune\r\n"), cmd->buffer[0] );
      validCmd = 1;
      break;
    case 'A': cc_tune_enable( 0 ); break;
      cc_tune_enable( 0 );

      cmd->n = sprintf_P( cmd->buffer, PSTR("# !%c Abort\r\n"), cmd->buffer[0] );
      validCmd = 1;
      break;
    case 'R':
	  cc_tune_enable( 0 );

      cc_cfg_default( CC1100_FREQ2, 3 );
      cc_cfg_get( CC1100_FREQ2, param, 3 );
      cc_param( CC1100_FREQ2, 3, param );
	  
      cmd->n = sprintf_P( cmd->buffer, PSTR("# !%c Reset F=%02x%02x%02x\r\n"), cmd->buffer[0], param[0],param[1],param[2] );
      validCmd = 1;
      break;
    case 'S':
      cc_param_read( CC1100_FREQ2, 3, param );
      cc_cfg_set( CC1100_FREQ2, param, 3 );
	
      cmd->n = sprintf_P( cmd->buffer, PSTR("# !%c Saved F=%02x%02x%02x\r\n"), cmd->buffer[0], param[0],param[1],param[2] );
      validCmd = 1;
    }
  } else {
	cc_param_read( CC1100_FREQ2, 3, param );
    cmd->n = sprintf_P( cmd->buffer, PSTR("# !%c"), cmd->buffer[0] );
	cmd->n += sprintf_P( cmd->buffer+cmd->n, PSTR(" F=%02x%02x%02x"), param[0],param[1],param[2] );
	if( cc_tuneEnabled() )
      cmd->n += sprintf_P( cmd->buffer+cmd->n, PSTR(" tuning"));
    cmd->n += sprintf_P( cmd->buffer+cmd->n, PSTR("\r\n"));
	validCmd = 1;
  }
  
  return validCmd;
}

static uint8_t cmd_eeprom(struct cmd *cmd) {
  uint8_t validCmd = 0;

  if( cmd->n > 1 ) {
    switch( cmd->buffer[1] & ~( 'A'^'a' ) ) {
    case 'R':
      nv_reset();

      cmd->n = sprintf_P( cmd->buffer, PSTR("# !%c reset\r\n"), cmd->buffer[0] );
      validCmd = 1;
    }
  }

  return validCmd;
}

//------------------------------------------------------------------------

static uint8_t check_command( struct cmd *cmd ) {
  uint8_t validCmd = 0;

  if( cmd->n > 0 ) {
    switch( cmd->buffer[0] & ~( 'A'^'a' ) ) {
    case 'V':  validCmd = cmd_version( cmd );       break;
    case 'T':  validCmd = cmd_trace( cmd );         break;
    case 'B':  validCmd = cmd_boot( cmd );          break;
    case 'C':  validCmd = cmd_cc1101( cmd );        break;
    case 'F':  validCmd = cmd_cc_tune( cmd );       break;
    case 'E':  validCmd = cmd_eeprom( cmd );        break;
    case 'I':  validCmd = cmd_id( cmd );            break;
    }
  }

  return validCmd;
}

uint8_t cmd( uint8_t byte, char **buffer, uint8_t *n ) {
  if( byte==CMD ) {
    reset_command();
    command.inCmd = 1;
  } else if( command.inCmd ) {
    if( byte=='\r' ) {
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

uint8_t cmd_str( char *str, char **buffer, uint8_t *n ) {
  uint8_t inCmd=0;
  uint8_t i;
  
  if( str ) {
    inCmd = 1;
    for( i=0 ; inCmd && str[i]!='\0' ; i++ )
      inCmd = cmd( str[i], buffer, n );
  }
  
  return inCmd;
}