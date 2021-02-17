/********************************************************
** message.c
**
** Packet conversion to message
**
********************************************************/
#include <string.h>
#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "trace.h"

#include "frame.h"
#include "message.h"

#define DEBUG_MSG(_v) DEBUG4(_v)

/********************************************************
** Message status
********************************************************/
enum message_state {
  S_START,
  S_HEADER,
  S_ADDR0,
  S_ADDR1,
  S_ADDR2,
  S_PARAM0,
  S_PARAM1,
  S_OPCODE,
  S_LEN,
  S_PAYLOAD,
  S_CHECKSUM,
  S_TRAILER,
  S_COMPLETE,
  S_ERROR
};

#define F_MASK  0x03
#define F_RQ    0x00
#define F_I     0x01
#define F_W     0x02
#define F_RP    0x03

#define F_ADDR0  0x10
#define F_ADDR1  0x20
#define F_ADDR2  0x40

#define F_PARAM0 0x04
#define F_PARAM1 0x08
#define F_RSSI   0x80

// Only used for received fields
#define F_OPCODE 0x01
#define F_LEN    0x02

#define MAX_RAW 162
#define MAX_PAYLOAD 64
struct message {
  uint8_t state;
  uint8_t count;

  uint8_t fields;  // Fields specified in header
  uint8_t rxFields;  // Fields actually received
  uint8_t error;

  uint8_t addr[3][3];
  uint8_t param[2];

  uint8_t opcode[2];
  uint8_t len;

  uint8_t csum;
  uint8_t rssi;

  uint8_t nPayload;
  uint8_t payload[MAX_PAYLOAD];

  uint8_t nBytes;
  uint8_t raw[MAX_RAW];
};

static void msg_reset( struct message *msg ) {
  if( msg != NULL ) {
    memset( msg, 0, sizeof(*msg) );
  }
}

/********************************************************
** Message lists
********************************************************/
#define N_MSG 4
#define N_LIST ( N_MSG+1 ) // Queues are bigger than total number of messsages so can never be full

struct msg_list {
  struct message *msg[N_LIST];
  uint8_t in;
  uint8_t out;
};

static void msg_put( struct msg_list *list, struct message **ppMsg, uint8_t reset ) {
  if( ppMsg != NULL ) {
    struct message *pMsg = (*ppMsg);
      if( pMsg != NULL ) {

      if( reset )
        msg_reset( pMsg );

      list->msg[list->in] = pMsg;
      list->in = ( list->in + 1 ) % N_LIST;
      (*ppMsg) = NULL;
    }
  }
}

static struct message *msg_get( struct msg_list *list ) {
  struct message *msg = list->msg[ list->out ];

  if( msg != NULL ) {
    list->msg[list->out] = NULL;
    list->out = ( list->out+1 ) % N_LIST;
    msg->state = S_START;
  }

  return msg;
}

/********************************************************
** Message structure pool
********************************************************/
static struct msg_list msg_pool;
void msg_free( struct message **msg ) { msg_put( &msg_pool, msg, 1 ); }
struct message *msg_alloc(void) {  return msg_get( &msg_pool); }

static void msg_create_pool(void) {
  static struct message MSG[N_MSG];
  uint8_t i;
  for( i=0 ; i<N_MSG ; i++ ) {
    struct message *msg = &MSG[i];
    msg_free( &msg );
  }
}

/********************************************************
** Received Message list
********************************************************/
static struct msg_list rx_list;
void msg_rx_ready( struct message **msg ) { msg_put( &rx_list, msg, 0 ); }
struct message *msg_rx_get(void) { return msg_get( &rx_list ); }


/********************************************************
** Transmit Message list
********************************************************/
static struct msg_list tx_list;
void msg_tx_ready( struct message **msg ) { msg_put( &tx_list, msg, 0 ); }
static struct message *msg_tx_get(void) {  return msg_get( &tx_list ); }

/********************************************************
** Message Header
********************************************************/
static char const * const MsgType[4] = { "RQ", "I","W","RP" };


#define F_OPTION ( F_ADDR0 + F_ADDR1 + F_ADDR2 + F_PARAM0 + F_PARAM1 )
#define F_MAND   ( F_OPCODE + F_LEN )

static const uint8_t address_flags[4] = {
  F_ADDR0+F_ADDR1+F_ADDR2 ,
                  F_ADDR2 ,
  F_ADDR0+        F_ADDR2 ,
  F_ADDR0+F_ADDR1
};

#define HDR_T_MASK 0x30
#define HDR_T_SHIFT   4
#define HDR_A_MASK 0x0C
#define HDR_A_SHIFT   2
#define HDR_PARAM0 0x02
#define HDR_PARAM1 0x01

static uint8_t get_hdr_flags(uint8_t header ) {
  uint8_t flags;

  flags = ( header & HDR_T_MASK ) >> HDR_T_SHIFT;   // Message type
  flags |= address_flags[ ( header & HDR_A_MASK ) >> HDR_A_SHIFT ];
  if( header & HDR_PARAM0 ) flags |= F_PARAM0;
  if( header & HDR_PARAM1 ) flags |= F_PARAM1;

  return flags;
}

static uint8_t get_header( uint8_t flags ) __attribute__((unused));
static uint8_t get_header( uint8_t flags ) {
  uint8_t i;

  uint8_t header = 0xFF;
  uint8_t addresses = flags & ( F_ADDR0+F_ADDR1+F_ADDR2 );

  for( i=0 ; i<sizeof(address_flags) ; i++ ) {
    if( addresses==address_flags[i] ) {
      header = i << HDR_A_SHIFT;
      header |= ( flags & F_MASK ) << HDR_T_SHIFT;  // Message type
      if( flags & F_PARAM0 ) header |= HDR_PARAM0;
      if( flags & F_PARAM1 ) header |= HDR_PARAM1;
      break;
    }
  }

  return header;
}

/********************************************************
** General utilities
********************************************************/
static uint8_t msg_checksum( struct message *msg ) {
  uint8_t csum;
  uint8_t i,j;
  
  // Missing fields will be zero so we can just add them to checksum without testing presence
                                                  { csum  = get_header(msg->fields); }
  for( i=0 ; i<3 ; i++ ) { for( j=0 ; j<3 ; j++ ) { csum += msg->addr[i][j]; }       }
  for( i=0 ; i<2 ; i++ )                          { csum += msg->param[i];           }
  for( i=0 ; i<2 ; i++ )                          { csum += msg->opcode[i];          }
                                                  { csum += msg->len;                }
  for( i=0 ; i<msg->nPayload ; i++ )              { csum += msg->payload[i];         }
  
  return -csum;
}

static void msg_set_address( uint8_t *addr, uint8_t class, uint32_t id ) {
  addr[0] = ( ( class<< 2 ) & 0xFC ) | ( ( id >> 16 ) & 0x03 );
  addr[1] =                            ( ( id >>  8 ) & 0xFF );
  addr[2] =                            ( ( id       ) & 0xFF );
}

static void msg_get_address( uint8_t *addr, uint8_t *class, uint32_t *id ) {
  uint8_t Class =          ( addr[0] & 0xFC ) >>  2;
  uint32_t Id =  (uint32_t)( addr[0] & 0x03 ) << 16
               | (uint32_t)( addr[1]        ) <<  8
               | (uint32_t)( addr[2]        )       ;

  if( class ) (*class) = Class;
  if( id    ) (*id   ) = Id;
}

/********************************************************
** Message Print
********************************************************/
static uint8_t msg_print_rssi( char *str, uint8_t rssi, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf_P(str, PSTR("%03u "), rssi );
  } else {
    n = sprintf_P(str, PSTR("--- "));
  }

  return n;
}

static uint8_t msg_print_type( char *str, uint8_t type ) {
  uint8_t n = 0;

 n = sprintf_P( str,PSTR("%2s "),MsgType[type] );

  return n;
}

static uint8_t msg_print_addr( char *str, uint8_t *addr, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    uint8_t class;
    uint32_t id;
	msg_get_address( addr, &class,&id );
	
    n = sprintf_P(str, PSTR("%02hu:%06lu "), class, id );
  } else {
    n = sprintf_P(str, PSTR("--:------ "));
  }

  return n;
}

static uint8_t msg_print_param( char *str, uint8_t param, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf_P(str, PSTR("%03u "), param );
  } else {
    n = sprintf_P(str, PSTR("--- "));
  }

  return n;
}

static uint8_t msg_print_opcode( char *str, uint8_t *opcode, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf_P( str, PSTR("%02X%02X "), opcode[0],opcode[1] );
  } else {
    n= sprintf_P(str, PSTR("???? "));
  }

  return n;
}

static uint8_t msg_print_len( char *str, uint8_t len, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf_P(str, PSTR("%03u "), len );
  } else {
    n = sprintf_P(str, PSTR("??? "));
  }

  return n;
}

static uint8_t msg_print_payload( char *str, uint8_t payload ) {
  uint8_t n=0;

  n = sprintf_P( str,PSTR("%02X"),payload );

  return n;
}


static uint8_t msg_print_error( char *str, uint8_t error ) {
  static char const msg_err_OK[] PROGMEM = "" ;
  static char const msg_err_UNKNOWN[] PROGMEM = "UNKNOWN" ;
#define _MSG_ERR(_e,_t) static char const msg_err_ ## _e[] PROGMEM = _t ;
  _MSG_ERR_LIST
#undef _MSG_ERR
#define _MSG_ERR(_e,_t) , msg_err_ ## _e
  static char const *const msg_err[MSG_ERR_MAX+1] PROGMEM = { msg_err_OK _MSG_ERR_LIST, msg_err_UNKNOWN };
#undef _MSG_ERR

  uint8_t n;

  if( error ) {
    if( error>MSG_ERR_MAX ) error = MSG_ERR_MAX;
    n = sprintf_P( str, PSTR(" * %S\r\n"), (PGM_P)pgm_read_word( msg_err + error ) );
  } else {
    n = sprintf_P(str,PSTR("\r\n"));
  }

  return n;
}

static uint8_t msg_print_raw( char *str, uint8_t raw, uint8_t i ) {
  uint8_t n = 0;

  if( i )
    n = sprintf_P( str,PSTR("%02X."),raw );
  else
    n = sprintf_P( str,PSTR("# %02X."),raw );

  return n;
}

static uint8_t msg_print_bytes( char *str, uint8_t raw, uint8_t i ) {
  uint8_t n = 0;

  if( i )
    n = sprintf_P( str,PSTR("%c"),raw );
  else
    n = sprintf_P( str,PSTR("# %c"),raw );

  return n;
}

/************************************************************************************
**
** msg_print_field
**
** get the next buffer of output text
**/

static uint8_t msg_print_field( struct message *msg, char *buff ) {
  uint8_t nBytes = 0;

  switch( msg->state ) {
  case S_START:
    nBytes = msg_print_rssi( buff, msg->rssi, msg->rxFields&F_RSSI );
    msg->state = S_HEADER;
    if( nBytes )
      break;
    /* fallthrough */

  case S_HEADER:
    nBytes = msg_print_type( buff, msg->fields & F_MASK );
    msg->state = S_PARAM0;
    if( nBytes )
      break;
    /* fallthrough */

  case S_PARAM0:
    nBytes = msg_print_param( buff, msg->param[0], msg->rxFields&F_PARAM0 );
    msg->state = S_ADDR0;
    if( nBytes )
      break;
    /* fallthrough */

  case S_ADDR0:
    nBytes = msg_print_addr( buff, msg->addr[0], msg->rxFields&F_ADDR0 );
    msg->state = S_ADDR1;
    if( nBytes )
      break;
    /* fallthrough */

  case S_ADDR1:
    nBytes = msg_print_addr( buff, msg->addr[1], msg->rxFields&F_ADDR1 );
    msg->state = S_ADDR2;
    if( nBytes )
      break;
    /* fallthrough */

  case S_ADDR2:
    nBytes = msg_print_addr( buff, msg->addr[2], msg->rxFields&F_ADDR2 );
    msg->state = S_OPCODE;
    if( nBytes )
      break;
    /* fallthrough */

  case S_OPCODE:
    nBytes = msg_print_opcode( buff, msg->opcode, msg->rxFields&F_OPCODE );
    msg->state = S_LEN;
    if( nBytes )
      break;
    /* fallthrough */

  case S_LEN:
    nBytes = msg_print_len( buff, msg->len, msg->rxFields&F_LEN );
    msg->state = S_PAYLOAD;
    if( nBytes )
      break;
    /* fallthrough */

  case S_PAYLOAD:
    // Multi buffer field
    if( msg->count < msg->nPayload ) {
      nBytes = msg_print_payload( buff, msg->payload[msg->count++] );
      if( nBytes )
        break;
    }

    msg->count = 0;
    msg->state = S_ERROR;
    /* fallthrough */

  case S_ERROR:
    // This always includes "\r\n"
    nBytes = msg_print_error( buff, msg->error );
    msg->state = S_TRAILER;
    if( nBytes )
      break;
    /* fallthrough */

  case S_TRAILER:   // Don't print trailer, use state for raw data
    // Multi buffer field
    if( msg->error || TRACE(TRC_RAW) ){
      if( msg->count < msg->nBytes ) {
        if( msg->rxFields&F_RSSI )
          nBytes = msg_print_raw( buff, msg->raw[msg->count], msg->count );
        else
          nBytes = msg_print_bytes( buff, msg->raw[msg->count], msg->count );
        msg->count++;
      } else if( msg->nBytes ) {
        nBytes = sprintf_P( buff, PSTR("\r\n") );
        msg->state = S_COMPLETE;
      }
      if( nBytes )
        break;
    }

    msg->count = 0;
    msg->state = S_COMPLETE;
    /* fallthrough */

  case S_COMPLETE:
    break;
  }

  return nBytes;
}

/************************************************************************************
**
** msg_print
**
** Called repeatedly from msg_work.
** Neither this function or any it calls must block or delay
**
** Acquires a buffer of output and tries to send it to the serial port
** If it's not transferred this time, try again until it goes.
**
** Keep getting more buffers until we get a zero length buffer
**
** We can re-use some of the working fields from struct message to control
** progress
**/

uint8_t msg_print( struct message *msg, char *msg_buff ) {
  uint8_t n;

  if( msg->state == S_START ) {
    DEBUG_MSG(1);
    msg->count = 0;
  }

  n = msg_print_field( msg, msg_buff );

  if( msg->state == S_COMPLETE )
    DEBUG_MSG(0);

  return n;
}

/********************************************************
** RX Message processing
********************************************************/
static uint8_t msg_rx_header( struct message *msg, uint8_t byte ) {
  uint8_t state = S_ADDR0;

  msg->fields = get_hdr_flags( byte );

  return state;
}

static uint8_t msg_rx_addr( struct message *msg, uint8_t addr, uint8_t byte ) {
  uint8_t state = S_ADDR0 + addr;

  msg->addr[addr][msg->count++] = byte;
  if( msg->count==sizeof( msg->addr[0] ) ) {
    msg->count = 0;
    state += 1;
    msg->rxFields |= F_ADDR0 << addr;
  }

  return state;
}

static uint8_t msg_rx_param( struct message *msg, uint8_t param, uint8_t byte ) {
  uint8_t state = S_PARAM0 + param;

  msg->param[param] = byte;
  state += 1;
  msg->rxFields |= F_PARAM0 << param;

  return state;
}

static uint8_t msg_rx_opcode( struct message *msg, uint8_t byte ) {
  uint8_t state = S_OPCODE;

  msg->opcode[msg->count++] = byte;
  if( msg->count==sizeof( msg->opcode ) ) {
    msg->count = 0;
    state += 1;
    msg->rxFields |= F_OPCODE ;
  }

  return state;
}

static uint8_t msg_rx_len( struct message *msg, uint8_t byte ) {
  uint8_t state = S_LEN;

  msg->len = byte;
  state += 1;
  msg->rxFields |= F_LEN;

  return state;
}

static uint8_t msg_rx_payload( struct message *msg, uint8_t byte ) {
  uint8_t state = S_PAYLOAD;

  if( msg->nPayload < MAX_PAYLOAD ) {
    msg->payload[msg->nPayload++] = byte;
  }

  msg->count++;
  if( msg->count==msg->len ) {
    msg->count = 0;
    state = S_CHECKSUM;
  }

  return state;
}

static uint8_t msg_rx_checksum( struct message *msg, uint8_t byte __attribute__((unused))) {
  uint8_t state = S_COMPLETE;

  if( msg->csum != 0 && !msg->error )
    msg->error = MSG_CSUM_ERR;

  return state;
}

static struct message *msgRx;
static void msg_rx_process(uint8_t byte) {
  msgRx->csum += byte;
  switch( msgRx->state ) {
    case S_START: // not processed here - fallthrough
    case S_HEADER:                                   { msgRx->state = msg_rx_header( msgRx, byte );   break; }
    case S_ADDR0:     if( msgRx->fields & F_ADDR0 )  { msgRx->state = msg_rx_addr( msgRx, 0, byte );  break; } /* fallthrough */
    case S_ADDR1:     if( msgRx->fields & F_ADDR1 )  { msgRx->state = msg_rx_addr( msgRx, 1, byte );  break; } /* fallthrough */
    case S_ADDR2:     if( msgRx->fields & F_ADDR2 )  { msgRx->state = msg_rx_addr( msgRx, 2, byte );  break; } /* fallthrough */
    case S_PARAM0:    if( msgRx->fields & F_PARAM0 ) { msgRx->state = msg_rx_param( msgRx, 0, byte ); break; } /* fallthrough */
    case S_PARAM1:    if( msgRx->fields & F_PARAM1 ) { msgRx->state = msg_rx_param( msgRx, 1, byte ); break; } /* fallthrough */
    case S_OPCODE:                                   { msgRx->state = msg_rx_opcode( msgRx, byte );   break; }
    case S_LEN:                                      { msgRx->state = msg_rx_len( msgRx, byte );      break; }
    case S_PAYLOAD:                                  { msgRx->state = msg_rx_payload( msgRx, byte );  break; }
    case S_CHECKSUM:                                 { msgRx->state = msg_rx_checksum( msgRx, byte ); break; }
  }
}

void msg_rx_rssi( uint8_t rssi ) {
  msgRx->rssi = rssi;
  msgRx->rxFields |= F_RSSI;
}

uint8_t *msg_rx_start(void) {
  uint8_t *raw = NULL;
  DEBUG_MSG(1);

  msgRx = msg_alloc();
  if( msgRx ) {
    raw = msgRx->raw;
    raw[0] = MAX_RAW;
  }

  DEBUG_MSG(0);

  return raw;
}

uint8_t msg_rx_byte( uint8_t byte ) {
  DEBUG_MSG(1);

  msg_rx_process( byte );

  DEBUG_MSG(0);

  return msgRx->error;
}

void msg_rx_end( uint8_t nBytes, uint8_t error ) {
  DEBUG_MSG(1);

  msgRx->nBytes = nBytes;

  if( error==MSG_OK ) {
    // All optional fields received as expected?
    if(   ( ( msgRx->rxFields & F_OPTION ) != ( msgRx->fields & F_OPTION ) )
       || ( ( msgRx->rxFields & F_MAND   ) != F_MAND  )
       || ( msgRx->len != msgRx->nPayload ) ) {
       error = MSG_TRUNC_ERR;
    }
  }

  msgRx->error = error;
  msg_rx_ready( &msgRx );

  DEBUG_MSG(0);
}

/********************************************************
** TX Message scan
********************************************************/
static uint8_t msg_scan_header( struct message *msg, char *str, uint8_t nChar ) {
  uint8_t ok = 0;
  uint8_t msgType;

  // Cheap conversion to upper for acceptable characters
  while( --nChar )
    str[ nChar-1 ] &= ~( 'A'^'a' );

  for( msgType=F_RQ ; msgType<=F_RP ; msgType++ ) {
    if( 0==strcmp( str, MsgType[msgType] ) ) {
      msg->fields = msgType;
	  ok = 1;
      break;
    }
  }

  return ok;
}

void msg_change_addr( struct message *msg, uint8_t addr, uint8_t Class,uint32_t Id , uint8_t myClass,uint32_t myId ) {
  if( msg && ( msg->fields & ( F_ADDR0 << addr ) ) ) { // Message contains specified address field
    uint8_t *Addr = msg->addr[addr];
	uint8_t class;
	uint32_t id;

	msg_get_address( Addr, &class, &id );
	if( class==Class && id==Id ) // address matches
	  msg_set_address( Addr, myClass, myId ); 
  }
}

static uint8_t msg_scan_addr( struct message *msg, char *str, uint8_t nChar ) {
  uint8_t ok = 0;
  uint8_t addr = msg->state - S_ADDR0;

  if( str[0]!='-' ) {
    uint8_t class;
    uint32_t id;

  	if( nChar<11 && 2==sscanf( str, "%hhu:%lu", &class, &id ) ) {
	  msg_set_address( msg->addr[addr], class, id );
      msg->fields |= F_ADDR0 << addr;
      ok = 1;
    } 
  } else {
    ok = 1;
  }

  return ok;
}

static uint8_t msg_scan_param( struct message *msg, char *str, uint8_t nChar ) {
  uint8_t ok = 0;
  uint8_t param = msg->state - S_PARAM0;

  if( str[0]!='-' ) {
  	if( nChar<5 && 1==sscanf( str, "%hhu", msg->param+param ) ) {
      msg->fields |= F_PARAM0 << param;
      ok = 1;
    } 
  } else {
    ok = 1;
  }

  return ok;
}

static uint8_t msg_scan_opcode( struct message *msg, char *str, uint8_t nChar ) {
  uint8_t ok = 0;

  if( nChar==5 && 2==sscanf( str, "%02hhx%02hhx", msg->opcode+0,msg->opcode+1 )  ) {
    msg->rxFields |= F_OPCODE;
    ok = 1;
  }

  return ok;
}

static uint8_t msg_scan_len( struct message *msg, char *str, uint8_t nChar ) {
  uint8_t ok = 0;

  if( nChar<5 && 1==sscanf( str, "%hhu", &msg->len ) ) {
    if( msg->len > 0 && msg->len <= MAX_PAYLOAD )
    {
      msg->rxFields |= F_LEN;
      ok = 1;
    }
  } 

  return ok;
}

static uint8_t msg_scan_payload( struct message *msg, char *str, uint8_t nChar ) {
  uint8_t ok=0;
  
  if( nChar==3 && 1==sscanf( str, "%02hhx", msg->payload+msg->nPayload ) )
  {
    msg->nPayload++;
	ok = 1;
  }

  return ok;
}

uint8_t msg_scan( struct message *msg, uint8_t byte ) {
  static char field[17];
  static uint8_t nChar=0;
  uint8_t ok = 1;

  if( byte=='\n' ) return 0; // Discard newline

  if( byte=='\r' ) {
    // Ignore blank line
    if( msg->state==S_START && nChar==0 )
      return 0;

    // Didn't get a sensible message
    if( msg->state != S_CHECKSUM ) {
      nChar = 0;
      msg->rxFields |= msg->fields;
	  msg->error = MSG_BAD_TX;
      return 1;
    } else {
      byte = '\0';
    }
  }

  if( msg->nBytes<MAX_RAW )
    msg->raw[msg->nBytes++] = byte;

  // Discard to end of line
  if( msg->state == S_ERROR )
    return 0;

  if( byte==' ' ) {
    // Discard leading spaces
    if( nChar==0 )
      return 0;

    // Terminate field
    byte = '\0';
  }
  field[ nChar++ ] = (char)byte;

  // No spaces between PAYLOAD bytes
  if( byte && msg->state == S_PAYLOAD ) {
    if( nChar==2 ) {
      field[nChar++] = '\0';

      ok = msg_scan_payload( msg, field, nChar );
      if( ok ) {
        nChar = 0;
        if( msg->nPayload == msg->len ) {
          msg->state = S_CHECKSUM;
        }
      }

    } else { // wait for second byte
      return 0;
    }
  }

  if( !byte ) {
    switch( msg->state ) {
    case S_START: /* fall through */
    case S_HEADER:      ok=msg_scan_header( msg, field, nChar ); msg->state = S_PARAM0;   break;
    case S_ADDR0:       ok=msg_scan_addr( msg, field, nChar );   msg->state = S_ADDR1;    break;
    case S_ADDR1:       ok=msg_scan_addr( msg, field, nChar );   msg->state = S_ADDR2;    break;
    case S_ADDR2:       ok=msg_scan_addr( msg, field, nChar );   msg->state = S_OPCODE;   break;
    case S_PARAM0:      ok=msg_scan_param( msg, field, nChar );  msg->state = S_ADDR0;    break;
//    case S_PARAM1:
    case S_OPCODE:      ok=msg_scan_opcode( msg, field, nChar ); msg->state = S_LEN;      break;
    case S_LEN:         ok=msg_scan_len( msg, field, nChar );    msg->state = S_PAYLOAD;  break;
    case S_PAYLOAD:                                              msg->state = S_ERROR;    break;
    case S_CHECKSUM:    msg->state = ( nChar!=1 ) ? S_ERROR : S_COMPLETE;                 break;
//  case S_TRAILER:
//  case S_COMPLETE:
//  case S_ERROR:
    }
    nChar = 0;
  }
  
  if( !ok )
    msg->state = S_ERROR;

  if( msg->state==S_PAYLOAD ) {
    if( ( msg->rxFields & F_MAND ) != F_MAND ) {
      msg->state = S_ERROR;
    }
  }

  if( msg->state==S_COMPLETE ) {
    msg->rxFields |= msg->fields;
    return 1;
  }

  return 0;
}

/********************************************************
** TX Message
********************************************************/
static uint8_t msg_tx_header( struct message *msg, uint8_t *done ) {
  uint8_t byte = 0;

  if( msg->count < 1 ) {
    byte = get_header( msg->fields );
    msg->count++;
  } else {
    msg->count = 0;
  }

  (*done) = (msg->count) ? 0:1;

  return byte;
}

static uint8_t msg_tx_addr( struct message *msg, uint8_t *done ) {
  uint8_t byte = 0;
  uint8_t addr = msg->state - S_ADDR0;

  if( msg->fields & ( F_ADDR0 << addr ) ) {
    if( msg->count < 3 ) {
      byte = msg->addr[ addr ][ msg->count ];
      msg->count++;
    } else {
      msg->count = 0;
    }
  }

  (*done) = (msg->count) ? 0:1;

  return byte;
}

static uint8_t msg_tx_param( struct message *msg, uint8_t *done  ) {
  uint8_t byte = 0;
  uint8_t param = msg->state - S_PARAM0;

  if( msg->fields & ( F_PARAM0 << param ) ) {
    if( msg->count < 1 ) {
      byte = msg->param[ param ];
      msg->count++;
    } else {
      msg->count = 0;
    }
  }

  (*done) = (msg->count) ? 0:1;

  return byte;
}

static uint8_t msg_tx_opcode( struct message *msg, uint8_t *done ) {
  uint8_t byte = 0;

  if( msg->count < 2 ) {
    byte = msg->opcode[ msg->count ];
    msg->count++;
  } else {
    msg->count = 0;
  }

  (*done) = (msg->count) ? 0:1;

  return byte;
}

static uint8_t msg_tx_len( struct message *msg, uint8_t *done ) {
  uint8_t byte = 0;

  if( msg->count < 1 ) {
    byte = msg->len;
    msg->count++;
  } else {
    msg->count = 0;
  }

  (*done) = (msg->count) ? 0:1;

  return byte;
}

static uint8_t msg_tx_payload( struct message *msg, uint8_t *done ) {
  uint8_t byte = 0;

  if( msg->count < msg->len ) {
    byte = msg->payload[ msg->count ];
    msg->count++;
  } else {
    msg->count = 0;
  }

  (*done) = (msg->count) ? 0:1;

  return byte;
}

static uint8_t msg_tx_checksum( struct message *msg, uint8_t *done ) {
  uint8_t byte = 0;

  if( msg->count < 1 ) {
    byte = msg->csum;
    msg->count++;
  } else {
    msg->count = 0;
  }

  (*done) = (msg->count) ? 0:1;

  return byte;
}

static uint8_t msg_tx_process( struct message *msg, uint8_t *done ) {
  uint8_t byte = 0, d=1;

  switch( msg->state ) {
  case S_START:
  case S_HEADER:     byte = msg_tx_header(msg,&d);    if( !d )break; msg->state = S_ADDR0;    /* fall through */
  case S_ADDR0:      byte = msg_tx_addr(msg,&d);      if( !d )break; msg->state = S_ADDR1;    /* fall through */
  case S_ADDR1:      byte = msg_tx_addr(msg,&d);      if( !d )break; msg->state = S_ADDR2;    /* fall through */
  case S_ADDR2:      byte = msg_tx_addr(msg,&d);      if( !d )break; msg->state = S_PARAM0;   /* fall through */
  case S_PARAM0:     byte = msg_tx_param(msg,&d);     if( !d )break; msg->state = S_PARAM1;   /* fall through */
  case S_PARAM1:     byte = msg_tx_param(msg,&d);     if( !d )break; msg->state = S_OPCODE;   /* fall through */
  case S_OPCODE:     byte = msg_tx_opcode(msg,&d);    if( !d )break; msg->state = S_LEN;      /* fall through */
  case S_LEN:        byte = msg_tx_len(msg,&d);       if( !d )break; msg->state = S_PAYLOAD;  /* fall through */
  case S_PAYLOAD:    byte = msg_tx_payload(msg,&d);   if( !d )break; msg->state = S_CHECKSUM; /* fall through */
  case S_CHECKSUM:   byte = msg_tx_checksum(msg,&d);  if( !d )break; msg->state = S_COMPLETE; /* fall through */
  case S_TRAILER:
  case S_COMPLETE:
  case S_ERROR:
    break;
  }

  (*done) = d;

  return byte;
}

static struct message *TxMsg;
static void msg_tx_start( struct message **msg ) {
  if( msg && (*msg) ) {
    TxMsg = (*msg);
	TxMsg->csum = msg_checksum( TxMsg );
    frame_tx_start( TxMsg->raw, MAX_RAW );
    (*msg) = NULL;
  }
}

uint8_t msg_tx_byte(uint8_t *done) {
  uint8_t byte=0x00;

  if( TxMsg ) {
    byte = msg_tx_process( TxMsg, done );
  } else {
    *done = 1;
  }

  return byte;
}

void msg_tx_end( uint8_t nBytes ) {
  if( TxMsg ) {
    TxMsg->nBytes = nBytes;
  }
}

void msg_tx_done(void) {
  if( TxMsg ) {
    // Make sure there's an RSSI value to print
    TxMsg->rxFields |= F_RSSI;
    TxMsg->rssi = 0;

    // Echo what we transmitted
    msg_rx_ready( &TxMsg );
  }
}

/************************************************************************************
**
** Message status functions
**/
uint8_t msg_isValid( struct message *msg ) {
  uint8_t isValid = 0;
  
  if( msg ) {
	isValid = ( msg->error==MSG_OK );
  }
  
  return isValid;
}

uint8_t msg_isTx( struct message *msg ) {
  uint8_t isTx = 0;
  
  if( msg ) {
	isTx = msg_isValid( msg ) && ( msg->rssi==0 );
  }

  return isTx;
}

/************************************************************************************
**
** msg_work must not block in any of it's activities
**
** Printing a recently received message should not prevent accepting serial input.
** Accepting serial input should not prevent us printing a new RX message
**
**/

void msg_work(void) {
 if( !TxMsg ) {
    struct message *tx1 = msg_tx_get();
    if( tx1 )
      msg_tx_start( &tx1 );
  }
}

/********************************************************
** System startup
********************************************************/

void msg_init( void ) {
  msg_create_pool();
}
