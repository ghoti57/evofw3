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

#include "message.h"

#define DEBUG_MSG(_v) DEBUG4(_v)

#define _MSG_ERR_LIST \
  _MSG_ERR( MSG_SIG_ERR,   "Bad Signature" ) \
  _MSG_ERR( MSG_MANC_ERR,  "Invalid Manchester Code" ) \
  _MSG_ERR( MSG_CSUM_ERR,  "Checksum error" ) \
  _MSG_ERR( MSG_TRUNC_ERR, "Truncated" ) \

#define _MSG_ERR(_e,_t) , _e
enum msg_err_code { MSG_OK _MSG_ERR_LIST, MSG_ERR_MAX };
#undef _MSG_ERR


/*******************************************************
* Manchester Encoding
*
* The [Evo Message] is encoded in the bitstream with a
* Manchester encoding.  This is the only part of the
* complete packet encoded in this way so we cannot
* use the built in function of the CC1101
*
* While the bitstream is interpreted as a Big-endian stream
* the manchester codes inserted in the stream are little-endian
*
* The Manchester data here is designed to correspond with the
* Big-endian byte stream seen in the bitstream.
*
********
* NOTE *
********
* The manchester decode process converts the data from
*     2x8 bit little-endian to 8 bit big-endian
* The manchester encode process converts the data from
*     8 bit big-endian to 2x8 bit little-endian
*
* Since only a small subset of 8-bit values are actually allowed in
* the bitstream rogue values can be used to identify some errors in
* the bitstream.
*
*/

// Convert big-endian 4 bits to little-endian byte
static uint8_t const man_encode[16] = {
  0xAA, 0xA9, 0xA6, 0xA5,  0x9A, 0x99, 0x96, 0x95,
  0x6A, 0x69, 0x66, 0x65,  0x5A, 0x59, 0x56, 0x55
};

// Convert little-endian 4 bits to 2-bit big endian
static uint8_t man_decode[16] = {
  0xF, 0xF, 0xF, 0xF, 0xF, 0x3, 0x2, 0xF,
  0xF, 0x1, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF
};

static inline int manchester_code_valid( uint8_t code ) {
 return ( man_decode[(code>>4)&0xF]!=0xF ) && ( man_decode[(code   )&0xF]!=0xF ) ;
}

static inline uint8_t manchester_decode( uint8_t byte ) {
  uint8_t decoded;

  decoded  = man_decode[( byte    ) & 0xF ];
  decoded |= man_decode[( byte>>4 ) & 0xF ]<<2;

  return decoded;
}

static inline void manchester_encode( uint8_t value, uint8_t *byte1, uint8_t *byte2 ) {
  *byte1 = man_encode[ ( value >> 4 ) & 0xF ];
  *byte2 = man_encode[ ( value      ) & 0xF ];
}


/********************************************************
** Message status
********************************************************/

#define MAX_PAYLOAD 64
struct message {
  uint8_t state;
  uint8_t count;

  uint8_t fields;  // Fields specified in header
  uint8_t rxFields;  // Fields actually received
  uint8_t error;

  uint8_t decoded;

  uint8_t addr[3][3];
  uint8_t param[2];

  uint8_t opcode[2];
  uint8_t len;

  uint8_t csum;
  uint8_t rssi;

  uint8_t nPayload;
  uint8_t payload[MAX_PAYLOAD];

  uint8_t nBytes;
  uint8_t raw[160];
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

static void msg_print_rssi( uint8_t rssi, uint8_t valid ) {
  if( valid ) {
    char str[8];
    sprintf(str, "%03u ", rssi );
    tty_write_str(str);
  } else {
    tty_write_str("--- ");
  }
}

static void msg_print_type( uint8_t type ) {
  static char const * const Type[4] = { "RQ ", " I "," W ","RP " };
  tty_write_str( Type[type] );
}

static void msg_print_addr( uint8_t *addr, uint8_t valid ) {
  if( valid ) {
    uint8_t  class = addr[0] >> 2;
    uint32_t dev = (uint32_t)( addr[0] & 0x03 ) << 16
                 | (uint32_t)( addr[1]        ) <<  8
                 | (uint32_t)( addr[2]        )       ;

    char str[12];
    sprintf(str, "%02hu:%06lu ", class, dev );
    tty_write_str(str);
  } else {
    tty_write_str("--:------ ");
  }
}

static void msg_print_param( uint8_t param, uint8_t valid ) {
  if( valid ) {
    char str[8];
    sprintf(str, "%03u ", param );
    tty_write_str(str);
  } else {
    tty_write_str("--- ");
  }
}

static void msg_print_opcode( uint8_t *opcode, uint8_t valid ) {
  if( valid ) {
    tty_write_hex( opcode[0] );
    tty_write_hex( opcode[1] );
    tty_write_char(' ');
  } else {
    tty_write_str("???? ");
  }
}

static void msg_print_len( uint8_t len, uint8_t valid ) {
  if( valid ) {
    char str[8];
    sprintf(str, "%03u ", len );
    tty_write_str(str);
  } else {
    tty_write_str("??? ");
  }
}

static void msg_print_payload( uint8_t *payload, uint8_t valid ) {
  while( valid ) {
    tty_write_hex( *payload );
    payload++;
    valid--;
  }
}

static void msg_print_error( uint8_t error ) {
#define _MSG_ERR(_e,_t) , _t
  static char const *const msg_err[MSG_ERR_MAX] = { "" _MSG_ERR_LIST };
#undef _MSG_ERR

  if( error ) {
    tty_write_str("* ");
    if( error < MSG_ERR_MAX )
      tty_write_str( msg_err[error] );
    else
      tty_write_str("UNKNOWN");
  }
}

static void msg_print_raw( uint8_t *raw, uint8_t nBytes ) {
  tty_write_str("# ");

  while( nBytes ) {
    tty_write_hex( *raw );tty_write_char('.');
    raw++;
    nBytes--;
  }

  tty_write_str("\r\n");
}

static void msg_print( struct message *msg ) {

  msg_print_rssi( msg->rssi, msg->rxFields&F_RSSI );
  msg_print_type( msg->fields & F_MASK );
  msg_print_param( msg->param[0], msg->rxFields&F_PARAM0 );
  msg_print_addr( msg->addr[0], msg->rxFields&F_ADDR0 );
  msg_print_addr( msg->addr[1], msg->rxFields&F_ADDR1 );
  msg_print_addr( msg->addr[2], msg->rxFields&F_ADDR2 );
  msg_print_opcode( msg->opcode, msg->rxFields&F_OPCODE );
  msg_print_len( msg->len, msg->rxFields&F_LEN );
  msg_print_payload( msg->payload, msg->nPayload );
  msg_print_error( msg->error );

  tty_write_str("\r\n");

  if( msg->error ) msg_print_raw( msg->raw, msg->nBytes );
}

/********************************************************
** Message structure pool
********************************************************/
static struct message *Msg[8];
static uint8_t msgIn=0;
static uint8_t msgOut=0;

static void msg_free( struct message *msg ) {
  if( msg != NULL ) {
    msg_reset( msg );

    Msg[ msgIn ] = msg;
    msgIn = ( msgIn+1 ) % 8;
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
  }

  return msg;
}

/********************************************************
** RX Message processing
********************************************************/
enum message_state {
  S_SIGNATURE,
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

static uint8_t msg_rx_signature( struct message *msg, uint8_t byte ) {
  static uint8_t const signature[] = { 0x33, 0x55, 0x53 };
  uint8_t state = S_SIGNATURE;

  // Validate it?
  msg->count++;
  if( msg->count==sizeof(signature) ) {
   state = S_HEADER;
   msg->count = 0;
  }

  return state;
}

static uint8_t msg_rx_header( struct message *msg, uint8_t byte ) {
  uint8_t state = S_ADDR0;

  msg->fields = get_hdr_flags( byte );
  if( hdr_param0(byte) ) msg->fields |= F_PARAM0;
  if( hdr_param1(byte) ) msg->fields |= F_PARAM1;

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
  uint8_t state = S_TRAILER;

  if( msg->csum != 0 && !msg->error )
    msg->error = MSG_CSUM_ERR;

  return state;
}

static uint8_t msg_rx_trailer( struct message *msg, uint8_t byte ) {
  static uint8_t trailer[] = { 0x35 };
  uint8_t state = S_TRAILER;

  // Validate it?
  msg->count++;
  if( msg->count==sizeof(trailer) ) {
    state = S_COMPLETE;
    msg->count = 0;
  }

  return state;
}

static struct message *msgRx;
static void msg_rx_process(uint8_t byte) {
  msgRx->raw[msgRx->nBytes] = byte;
  msgRx->nBytes++;

  if( msgRx->state == S_SIGNATURE )
    msgRx->state = msg_rx_signature( msgRx, byte );
  else if( msgRx->state == S_TRAILER )
    msgRx->state = msg_rx_trailer( msgRx, byte );
  else if( msgRx->state != S_TRAILER ) {
    // Bytes here come in Manchester code pairs
    if( !manchester_code_valid(byte) ) msgRx->error = MSG_MANC_ERR;
    msgRx->decoded |= manchester_decode(byte);
    if( !( msgRx->nBytes & 1 ) )
      msgRx->decoded <<= 4;
    else  {
      msgRx->csum += msgRx->decoded;
      switch( msgRx->state ) {
        case S_HEADER:                                   { msgRx->state = msg_rx_header( msgRx, msgRx->decoded );   break; }
        case S_ADDR0:     if( msgRx->fields & F_ADDR0 )  { msgRx->state = msg_rx_addr( msgRx, 0, msgRx->decoded );  break; } /* fallthrough */
        case S_ADDR1:     if( msgRx->fields & F_ADDR1 )  { msgRx->state = msg_rx_addr( msgRx, 1, msgRx->decoded );  break; } /* fallthrough */
        case S_ADDR2:     if( msgRx->fields & F_ADDR2 )  { msgRx->state = msg_rx_addr( msgRx, 2, msgRx->decoded );  break; } /* fallthrough */
        case S_PARAM0:    if( msgRx->fields & F_PARAM0 ) { msgRx->state = msg_rx_param( msgRx, 0, msgRx->decoded ); break; } /* fallthrough */
        case S_PARAM1:    if( msgRx->fields & F_PARAM1 ) { msgRx->state = msg_rx_param( msgRx, 1, msgRx->decoded ); break; } /* fallthrough */
        case S_OPCODE:                                   { msgRx->state = msg_rx_opcode( msgRx, msgRx->decoded );   break; }
        case S_LEN:                                      { msgRx->state = msg_rx_len( msgRx, msgRx->decoded );      break; }
        case S_PAYLOAD:                                  { msgRx->state = msg_rx_payload( msgRx, msgRx->decoded );  break; }
        case S_CHECKSUM:                                 { msgRx->state = msg_rx_checksum( msgRx, msgRx->decoded ); break; }
      }
      msgRx->decoded   = 0;
    }
  }
}

void msg_rx_rssi( uint8_t rssi ) {
  msgRx->rssi = rssi;
  msgRx->rxFields |= F_RSSI;
}

static void msg_rx_start(void) {
  msgRx = msg_alloc();
}

static void msg_rx_end(void) {
  // All optional fields received as expected
  if(   ( ( msgRx->rxFields & F_OPTION ) != ( msgRx->fields & F_OPTION ) )
     || ( ( msgRx->rxFields & F_MAND   ) != F_MAND  )
     || ( msgRx->len != msgRx->nPayload ) ) {
    msgRx->error = MSG_TRUNC_ERR;
  }

  msg_ready( msgRx );
  msgRx = NULL;
}


void msg_rx_byte( uint8_t byte ) {
  DEBUG_MSG(1);

  if( byte==MSG_START ) {
    msg_rx_start();
  }  else if( msgRx ) {
    if( byte==MSG_END ) {
      msg_rx_end();
    } else if( byte==0x00 ) {
      msg_rx_rssi(0);
      msg_rx_end();
      msg_rx_start();
    } else {
      msg_rx_process( byte );
    }
  }

  DEBUG_MSG(0);
}

void msg_work(void) {
  struct message *msg = msg_get();
  if( msg != NULL ) {
    msg_print( msg );
    msg_free( msg );
  }
}

/********************************************************
** System startup
********************************************************/

static void msg_create_pool(void) {
  static struct message MSG[4];
  uint8_t i;
  for( i=0 ; i< 4; i++ )
   msg_free( MSG+i );
}

void msg_init(void) {
  msg_create_pool();
}
