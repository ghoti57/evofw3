/********************************************************
** message.c
**
** Packet conversion to message
**
********************************************************/
#include <string.h>
#include <stdio.h>

#include <avr/interrupt.h>

#include "config.h"
#include "tty.h"
#include "trace.h"
#include "cmd.h"

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
** Message Header
********************************************************/

#define F_MASK  0x03
#define F_RQ    0x00
#define F_I     0x01
#define F_W     0x02
#define F_RP    0x03
inline uint8_t pkt_type(uint8_t flags) { return flags & F_MASK; }
inline void set_request(uint8_t *flags)      { *flags = F_RQ; }
inline void set_information(uint8_t *flags)  { *flags = F_I; }
inline void set_write(uint8_t *flags)        { *flags = F_W; }
inline void set_response(uint8_t *flags)     { *flags = F_RP; }

#define F_ADDR0  0x10
#define F_ADDR1  0x20
#define F_ADDR2  0x40

#define F_PARAM0 0x04
#define F_PARAM1 0x08
#define F_RSSI   0x80

// Only used for received fields
#define F_OPCODE 0x01
#define F_LEN    0x02

#define F_OPTION ( F_ADDR0 + F_ADDR1 + F_ADDR2 + F_PARAM0 + F_PARAM1 )
#define F_MAND   ( F_OPCODE + F_LEN )

static const uint8_t header_flags[16] = {
  F_RQ + F_ADDR0+F_ADDR1+F_ADDR2 ,
  F_RQ +                 F_ADDR2 ,
  F_RQ + F_ADDR0+        F_ADDR2 ,
  F_RQ + F_ADDR0+F_ADDR1         ,
  F_I  + F_ADDR0+F_ADDR1+F_ADDR2 ,
  F_I  +                 F_ADDR2 ,
  F_I  + F_ADDR0+        F_ADDR2 ,
  F_I  + F_ADDR0+F_ADDR1         ,
  F_W  + F_ADDR0+F_ADDR1+F_ADDR2 ,
  F_W  +                 F_ADDR2 ,
  F_W  + F_ADDR0+        F_ADDR2 ,
  F_W  + F_ADDR0+F_ADDR1         ,
  F_RP + F_ADDR0+F_ADDR1+F_ADDR2 ,
  F_RP +                 F_ADDR2 ,
  F_RP + F_ADDR0+        F_ADDR2 ,
  F_RP + F_ADDR0+F_ADDR1         ,
};

static uint8_t get_hdr_flags(uint8_t header ) {
  return header_flags[ ( header>>2 ) & 0x0F ];
}

static uint8_t get_header( uint8_t flags ) {
  uint8_t i;

  for( i=0 ; i<sizeof(header_flags) ; i++ ) {
    if( flags==header_flags[i] )
      break;
  }

  return i<<2;  // Will return 0x40 if not found
}

#define HDR_PARAM0 0x02
#define HDR_PARAM1 0x01

inline uint8_t hdr_param0(uint8_t header)    { return header & HDR_PARAM0; }
inline uint8_t hdr_param1(uint8_t header)    { return header & HDR_PARAM1; }

/********************************************************
** Message Print
********************************************************/
static uint8_t msg_print_rssi( char *str, uint8_t rssi, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf(str, "%03u ", rssi );
  } else {
    n = sprintf(str, "--- ");
  }

  return n;
}

static uint8_t msg_print_type( char *str, uint8_t type ) {
  static char const * const Type[4] = { "RQ ", " I "," W ","RP " };
  uint8_t n = 0;

  n = sprintf( str, Type[type] );

  return n;
}

static uint8_t msg_print_addr( char *str, uint8_t *addr, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    uint8_t  class =         ( addr[0] & 0xFC ) >>  2;
    uint32_t dev = (uint32_t)( addr[0] & 0x03 ) << 16
                 | (uint32_t)( addr[1]        ) <<  8
                 | (uint32_t)( addr[2]        )       ;

    n = sprintf(str, "%02hu:%06lu ", class, dev );
  } else {
    n = sprintf(str, "--:------ ");
  }

  return n;
}

static uint8_t msg_print_param( char *str, uint8_t param, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf(str, "%03u ", param );
  } else {
    n = sprintf(str, "--- ");
  }

  return n;
}

static uint8_t msg_print_opcode( char *str, uint8_t *opcode, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf( str, "%02X%02X ", opcode[0],opcode[1] );
  } else {
    n= sprintf(str, "???? ");
  }

  return n;
}

static uint8_t msg_print_len( char *str, uint8_t len, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf(str, "%03u ", len );
  } else {
    n = sprintf(str, "??? ");
  }

  return n;
}

static uint8_t msg_print_payload( char *str, uint8_t payload ) {
  uint8_t n=0;

  n = sprintf( str,"%02X",payload );

  return n;
}


static uint8_t msg_print_error( char *str, uint8_t error ) {
#define _MSG_ERR(_e,_t) , _t
  static char const *const msg_err[MSG_ERR_MAX] = { "" _MSG_ERR_LIST };
#undef _MSG_ERR
    uint8_t n;

    if( error )
      n = sprintf( str," * %s\r\n", ( error < MSG_ERR_MAX ) ? msg_err[error] : "UNKNOWN" );
    else
      n = sprintf( str,"\r\n" );

    return n;
  }

static uint8_t msg_print_raw( char *str, uint8_t raw, uint8_t i ) {
  uint8_t n = 0;

  if( i )
    n = sprintf( str,"%02X.",raw );
  else
    n = sprintf( str,"# %02X.",raw );

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
        nBytes = msg_print_raw( buff, msg->raw[msg->count], msg->count );
        msg->count++;
      } else if( msg->nBytes ) {
        nBytes = sprintf( buff, "\r\n" );
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
** Try to catch suspect messages for debug purposes
*/
static uint8_t check_payload_2309( struct message *msg ) {
  uint8_t i;

  for( i=0 ; i<msg->len ; i+=3 ) {
    if( msg->payload[i] > 11 ) return MSG_SUSPECT_WARN;  // Bad zone number
  }

  return MSG_OK;
}

static uint8_t msg_check_payload( struct message *msg ) {
  uint8_t error = MSG_OK;

  uint16_t opcode = ( msg->opcode[0]<<8 ) + msg->opcode[1];
  switch( opcode ) {
  case 0x2309: error = check_payload_2309(msg); break;
  }

  return error;
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

static uint8_t msg_print( struct message *msg ) {
  static char msg_buff[TXBUF];
  static uint8_t n;

  if( msg->state == S_START ) {
    DEBUG_MSG(1);
    msg->count = 0;
    n = 0;

    if( msg->error==MSG_OK )
      msg->error = msg_check_payload( msg );
  }

  // Do we still have outstanding text to send?
  if( n ) {
    n -= tty_put_str( (uint8_t *)msg_buff, n );
  }

  if( !n ) {
    n = msg_print_field( msg, msg_buff );
  }

  if( msg->state == S_COMPLETE )
    DEBUG_MSG(0);

  return n;
}

/********************************************************
** Message structure pool
********************************************************/
static struct message *Msg[8];
static uint8_t msgIn=0;
static uint8_t msgOut=0;

static void msg_free( struct message **msg ) {
  if( msg!=NULL && (*msg)!=NULL ) {
    msg_reset( (*msg) );

    Msg[ msgIn ] = (*msg);
    msgIn = ( msgIn+1 ) % 8;

    (*msg) = NULL;
  }
}

static struct message *msg_alloc(void) {
  struct message *msg = Msg[ msgOut ];
  if( msg != NULL ) {
    Msg[msgOut] = NULL;
    msgOut = ( msgOut+1 )  % 8;
  }

  return msg;
}

/********************************************************
** Received Message list
********************************************************/
static struct message *MsgRx[8];
static uint8_t msgRxIn=0;
static uint8_t msgRxOut=0;

static void msg_ready( struct message *msg ) {
  if( msg != NULL ) {
    MsgRx[ msgRxIn ] = msg;
    msgRxIn = ( msgRxIn+1 ) % 8;
  }
}

static struct message *msg_get(void) {
  struct message *msg = MsgRx[ msgRxOut ];
  if( msg != NULL ) {
    MsgRx[msgRxOut] = NULL;
    msgRxOut = ( msgRxOut+1 ) % 8;
    msg->state = S_START;
  }

  return msg;
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

static uint8_t msg_rx_checksum( struct message *msg, uint8_t byte ) {
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

  msg_ready( msgRx );
  msgRx = NULL;

  DEBUG_MSG(0);
}

/************************************************************************************
**
** msg_work must not block in any of it's activities
**
** Printing a recently received message should not prevent accepting serial input.
** Accepting serial input should not prevent us printing a new RX message
**
**/

static uint8_t inCmd;
static uint8_t *cmdBuff;
static uint8_t nCmd;

void msg_work(void) {
  static struct message *rx = NULL;
  static struct message *tx = NULL;

  uint8_t byte;

  // Print RX messages
  if( rx ) {
    if( !msg_print( rx ) ) {
      msg_free( &rx );
    }
  } else if( nCmd ) {
    nCmd -= tty_put_str( cmdBuff, nCmd );
    if( !nCmd )
      inCmd = 0;
  } else {
    // If we have a message now we'll start printing it next time
    rx = msg_get();
  }

  // Process serial data from host
  if( !tx ) tx = msg_alloc();

  byte = tty_rx_get();
  if( byte ) {
    if( !tx || tx->state==S_START ) {
      if( !nCmd && ( byte==CMD || inCmd ) ) {
        inCmd = cmd( byte, &cmdBuff, &nCmd );
      }
    }
  }
}

/********************************************************
** System startup
********************************************************/
static void msg_create_pool(void) {
  static struct message MSG[4];
  uint8_t i;
  for( i=0 ; i< 4; i++ ) {
    struct message *msg = MSG+i;
    msg_free( &msg );
  }
}

void msg_init(void) {
  msg_create_pool();

  // Force a version string to be printed
  inCmd = cmd(CMD, NULL,NULL );
  inCmd = cmd('V', NULL,NULL );
  inCmd = cmd('\n', &cmdBuff, &nCmd );
}
