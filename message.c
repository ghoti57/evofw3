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
  0x55, 0x95, 0x65, 0xA5, 0x59, 0x99, 0x69, 0xA9,
  0x56, 0x96, 0x66, 0xA6, 0x5A, 0x9A, 0x6A, 0xAA 
};

// Convert little-endian 4 bits to 2-bit big endian
static uint8_t man_decode[16] = {
  0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0x2, 0xF,
  0xF, 0x1, 0x3, 0xF, 0xF, 0xF, 0xF, 0xF 
};

static inline int manchester_code_valid( uint8_t code ) { 
 return ( man_decode[(code>>4)&0xF]!=0xF ) && ( man_decode[(code   )&0xF]!=0xF ) ; 
}

static inline uint8_t manchester_decode( uint8_t byte ) {
  uint8_t decoded;
  
  decoded  = man_decode[( byte    ) & 0xF ]<<2;
  decoded |= man_decode[( byte>>4 ) & 0xF ];

  return decoded;
}

static inline uint8_t manchester_encode( uint8_t value ) {
  return man_encode[ value & 0xF ];
}


/********************************************************
** Message status
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

#define F_MASK   0x03
#define F_RQ     0x00
#define F_I	     0x01
#define F_W      0x02
#define F_RP     0x03

#define F_ADDR0  0x10
#define F_ADDR1  0x20
#define F_ADDR2  0x40

#define F_PARAM0 0x04
#define F_PARAM1 0x08
#define F_RSSI   0x80

// Only used for received fields
#define F_OPCODE 0x01
#define F_LEN    0x02

#define MAX_PAYLOAD 64
struct message {
  uint8_t state;
  uint8_t count;
  
  uint8_t fields;	// Fields specified in header
  uint8_t rxFields;	// Fields actually received
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
** Message structure pool
********************************************************/
static struct message *Msg[8];
static uint8_t msgIn=0;
static uint8_t msgOut=0;

static void msg_free( struct message **msg ) {
  if( msg && (*msg) ) {
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

static void msg_rx_ready( struct message **msg ) {
  if( msg && (*msg) ) {
    MsgRx[ msgRxIn ] = (*msg);
    msgRxIn = ( msgRxIn+1 ) % 8;
	(*msg) = NULL;
  }
}

static struct message *msg_rx_get(void) {
  struct message *msg = MsgRx[ msgRxOut ];
  if( msg != NULL ) {
    MsgRx[msgRxOut] = NULL;
    msgRxOut = ( msgRxOut+1 ) % 8;
    msg->state = S_SIGNATURE;
  }
  
  return msg;
}

/********************************************************
** Transmit Message list
********************************************************/
static struct message *MsgTx[8];
static uint8_t msgTxIn=0;
static uint8_t msgTxOut=0;

static void msg_tx_ready( struct message **msg ) {
  if( msg && (*msg) ) {
    MsgTx[ msgTxIn ] = (*msg);
    msgTxIn = ( msgTxIn+1 ) % 8;
	(*msg) = NULL;
  }
}

struct message *msg_tx_get(void) {
  struct message *msg = MsgTx[ msgTxOut ];
  if( msg != NULL ) {
    msg->state = S_SIGNATURE;
    MsgTx[msgTxOut] = NULL;
    msgTxOut = ( msgTxOut+1 ) % 8;
  }
  
  return msg;
}

void msg_tx_done( struct message **msg ) {
  // Make sure there's an RSSI value to print
  (*msg)->rxFields |= F_RSSI;
  (*msg)->rssi = 0;

  // Echo what we transmitted
  msg_rx_ready( msg );	
}

/********************************************************
** Message Header
********************************************************/

inline uint8_t pkt_type(uint8_t flags) { return flags & F_MASK; }
inline void set_request(uint8_t *flags)     { *flags = F_RQ; }
inline void set_information(uint8_t *flags) { *flags = F_I; }
inline void set_write(uint8_t *flags)       { *flags = F_W; }
inline void set_response(uint8_t *flags)    { *flags = F_RP; }

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

  return i<<2;	// Will return 0x40 if not found
}
  
#define HDR_PARAM0 0x02
#define HDR_PARAM1 0x01
inline uint8_t hdr_param0(uint8_t header)    { return header & HDR_PARAM0; }
inline uint8_t hdr_param1(uint8_t header)    { return header & HDR_PARAM1; }

/********************************************************
** Message fields
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

static uint8_t const signature[] = { 0xCC, 0xAA, 0xCA };
static uint8_t msg_rx_signature( struct message *msg, uint8_t byte __attribute__ ((unused)) ) {
  uint8_t state = S_SIGNATURE;
  
  // Validate it?
  msg->count++;
  if( msg->count==sizeof(signature) ) {
	 state = S_HEADER;
	 msg->count = 0;
  }
  return state;
}

static uint8_t msg_tx_signature( struct message *msg ) {
  uint8_t byte = 0;

  if( msg->count < sizeof(signature) ) {
    byte = signature[ msg->count ];
	msg->count++;
  } else {
	msg->count = 0;
  }
  
  return byte;
}

//******************************************************************

static char const * const Type[4] = { "RQ","I","W","RP" };

static uint8_t msg_print_type( char *str, uint8_t type ) {
  uint8_t n = 0;

  if( type <= F_RP )
    n = sprintf( str, "%2s ", Type[type] );
  else
    n = sprintf( str, "?? ");

  return n;
}

static void msg_scan_header( struct message *msg, char *str, uint8_t nChar ) {
  uint8_t msgType;
  // Cheap conversion to upper for acceptable characters
  while( --nChar )
    str[ nChar-1 ] &= ~( 'A'^'a' );

  for( msgType=F_RQ ; msgType<=F_RP ; msgType++ ) {
    if( !strcmp( str, Type[msgType] ) ) {
      msg->fields = msgType << 2;
      break;
    }
  }
  msg_print_type( str, msg->fields );
}

static uint8_t msg_rx_header( struct message *msg, uint8_t byte ) {
  uint8_t state = S_ADDR0;

  msg->fields = get_hdr_flags( byte );
  if( hdr_param0(byte) ) msg->fields |= F_PARAM0;
  if( hdr_param1(byte) ) msg->fields |= F_PARAM1;

  return state;
}

static uint8_t msg_tx_header( struct message *msg ) {
  uint8_t byte = 0;

  if( msg->count < 2 ) {
	uint8_t raw = get_header( msg->fields );
    byte = manchester_encode( ( msg->count & 1 ) ? raw : raw >> 4 );
	msg->count++;
  } else {
	msg->count = 0;
  }
  
  return byte;
}

//******************************************************************

static uint8_t  myClass = 18;
static uint32_t myId = 0x4DADA;

static uint8_t msg_print_addr( char *str, uint8_t *addr, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
	uint8_t  class = addr[0] >> 2;
	uint32_t dev = (uint32_t)( addr[0] & 0x03 ) << 16
	             | (uint32_t)( addr[1]        ) <<  8
	             | (uint32_t)( addr[2]        )       ;

    n = sprintf(str, "%02hu:%06lu ", class, dev );
  } else {
	n = sprintf(str, "--:------ ");
  }
  
  return n;
}

static void msg_scan_addr( struct message *msg, char *str, uint8_t nChar __attribute__ ((unused)) ) {
  uint8_t ok = 0;
  uint8_t addr = msg->state - S_ADDR0;
  uint8_t class;
  uint32_t id;
  
  if( str[0]!='-' && 2 == sscanf( str, "%hhu:%lu", &class, &id ) ) {
	// Specific address for thios device
	if( class==18 && id ==730 ) {
	  class = myClass;
	  id = myId;
	}

	msg->addr[addr][0] = ( class<< 2 ) | ( ( id >> 16 ) & 0x03 );
	msg->addr[addr][1] =                 ( ( id >>  8 ) & 0xFF );
	msg->addr[addr][2] =                 ( ( id       ) & 0xFF );
	
	msg->fields |= F_ADDR0 << addr;
	ok = 1;
	
	msg->csum += msg->addr[addr][0] + msg->addr[addr][1] + msg->addr[addr][2];
  }

  msg_print_addr( str, msg->addr[addr], ok );
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

static uint8_t msg_tx_addr( struct message *msg ) {
  uint8_t byte = 0;
  uint8_t addr = msg->state - S_ADDR0;
  
  if( msg->fields & ( F_ADDR0 << addr ) ) {
    if( msg->count < 6 ) {
	  uint8_t raw = msg->addr[ addr ][ msg->count/2 ];
      byte = manchester_encode( ( msg->count & 1 ) ? raw : raw >> 4 );
	  msg->count++;
    } else {
	  msg->count = 0;
    }
  }
  
  return byte;
}

//******************************************************************

static uint8_t msg_print_param( char *str, uint8_t param, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf(str, "%03u ", param );
  } else {
	n = sprintf(str, "--- ");
  }
  
  return n;
}

static void msg_scan_param( struct message *msg, char *str, uint8_t nChar __attribute__ ((unused)) ) {
  uint8_t ok = 0;
  uint8_t param = msg->state - S_PARAM0;

  if( str[0]!='-' && 1 == sscanf( str, "%hhu", msg->param+param ) ) {
	msg->fields |= F_PARAM0 << param;
	ok = 1;
	
    msg->csum += msg->param[param];
  }

  msg_print_param( str, msg->param[param], ok );
}

static uint8_t msg_rx_param( struct message *msg, uint8_t param, uint8_t byte ) {
  uint8_t state = S_PARAM0 + param;

  msg->param[param] = byte;
  state += 1;
  msg->rxFields |= F_PARAM0 << param;
  
  return state;
}

static uint8_t msg_tx_param( struct message *msg ) {
  uint8_t byte = 0;
  uint8_t param = msg->state - S_PARAM0;

  if( msg->fields & ( F_PARAM0 << param ) ) {
    if( msg->count < 2 ) {
	  uint8_t raw = msg->param[ param ];
      byte = manchester_encode( ( msg->count & 1 ) ? raw : raw >> 4 );
	  msg->count++;
    } else {
	  msg->count = 0;
    }
  }
  
  return byte;
}

//******************************************************************

static uint8_t msg_print_opcode( char *str, uint8_t *opcode, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
	n = sprintf( str, "%02hhX%02hhX ", opcode[0],opcode[1] );
  } else {
	n= sprintf(str, "???? ");
  }
  
  return n;
}

static void msg_scan_opcode( struct message *msg, char *str, uint8_t nChar __attribute__ ((unused)) ) {
  uint8_t ok = 0;
  
  if( str[0]!='-' && 2 == sscanf( str, "%02hhx%02hhx", msg->opcode+0,msg->opcode+1 )  ) {
	msg->rxFields |= F_OPCODE;
	ok = 1;
	
    msg->csum += msg->opcode[0] + msg->opcode[1];
  }
  
  msg_print_opcode( str, msg->opcode, ok );
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

static uint8_t msg_tx_opcode( struct message *msg ) {
  uint8_t byte = 0;

  if( msg->count < 4 ) {
	uint8_t raw = msg->opcode[ msg->count/2 ];
    byte = manchester_encode( ( msg->count & 1 ) ? raw : raw >> 4 );
	msg->count++;
  } else {
	msg->count = 0;
  }
  
  return byte;
}

//******************************************************************

static uint8_t msg_print_len( char *str, uint8_t len, uint8_t valid ) {
  uint8_t n = 0;

  if( valid ) {
    n = sprintf(str, "%03u ", len );
  } else {
	n = sprintf(str, "??? ");
  }
  
  return n;
}

static void msg_scan_len( struct message *msg, char *str, uint8_t nChar __attribute__ ((unused)) ) {
  uint8_t ok = 0;
  
  if( str[0]!='-' && 1 == sscanf( str, "%hhu", &msg->len ) ) {
	if( msg->len > 0 && msg->len <= MAX_PAYLOAD )
	{
      msg->rxFields |= F_LEN;
      ok = 1;
	  
	  msg->csum += msg->len;
	}
  }

  msg_print_len( str, msg->len, ok );
}

static uint8_t msg_rx_len( struct message *msg, uint8_t byte ) {
  uint8_t state = S_LEN;

  msg->len = byte;
  state += 1;
  msg->rxFields |= F_LEN;
  
  return state;
}

static uint8_t msg_tx_len( struct message *msg ) {
  uint8_t byte = 0;

  if( msg->count < 2 ) {
	uint8_t raw = msg->len;
    byte = manchester_encode( ( msg->count & 1 ) ? raw : raw >> 4 );
	msg->count++;
  } else {
	msg->count = 0;
  }
  
  return byte;
}

//******************************************************************

static uint8_t msg_print_payload( char *str, uint8_t payload ) {
  uint8_t n=0;
  
  n = sprintf( str,"%02X",payload ); 

  return n;
}

static void msg_scan_payload( struct message *msg, char *str, uint8_t nChar __attribute__ ((unused)) ) {

  if( 1 == sscanf( str, "%02hhx", msg->payload+msg->nPayload ) )
  {
    msg->csum += msg->payload[msg->nPayload];
    msg_print_payload( str, msg->payload[msg->nPayload] );
  }
  else
    sprintf( str,"??" );
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

static uint8_t msg_tx_payload( struct message *msg ) {
  uint8_t byte = 0;

  if( msg->count < 2*msg->len ) {
	uint8_t raw = msg->payload[ msg->count/2 ];
    byte = manchester_encode( ( msg->count & 1 ) ? raw : raw >> 4 );
	msg->count++;
  } else {
	msg->count = 0;
  }
  
  return byte;
}

//******************************************************************

static uint8_t msg_rx_checksum( struct message *msg, uint8_t byte __attribute__ ((unused)) ) {
  uint8_t state = S_TRAILER;

  if( msg->csum != 0 && !msg->error )
	msg->error = MSG_CSUM_ERR;

  return state;
}

static uint8_t msg_tx_checksum( struct message *msg ) {
  uint8_t byte = 0;

  if( msg->count < 2 ) {
	uint8_t raw = msg->csum;
    byte = manchester_encode( ( msg->count & 1 ) ? raw : raw >> 4 );
	msg->count++;
  } else {
	msg->count = 0;
  }
  
  return byte;
}

//******************************************************************

static uint8_t trailer[] = { 0xAC };
static uint8_t msg_rx_trailer( struct message *msg, uint8_t byte __attribute__ ((unused)) ) {
  uint8_t state = S_TRAILER;
  
  // Validate it?
  msg->count++;
  if( msg->count==sizeof(trailer) ) {
	 state = S_COMPLETE;
	 msg->count = 0;
  }
  
  return state;
}

static uint8_t msg_tx_trailer( struct message *msg ) {
  uint8_t byte = 0;

  if( msg->count < sizeof(trailer) ) {
    byte = trailer[ msg->count ];
	msg->count++;
  } else {
	msg->count = 0;
  }
  
  return byte;
}

//******************************************************************

static uint8_t msg_print_error( char *str, uint8_t error ) {
#define _MSG_ERR(_e,_t) , _t
 static char const *const msg_err[MSG_ERR_MAX] = { "" _MSG_ERR_LIST };
#undef _MSG_ERR
  uint8_t n = 0;

  if( error )
    n = sprintf( str," * %s\r\n", ( error < MSG_ERR_MAX ) ? msg_err[error] : "UNKNOWN" );
  else 
    n = sprintf( str,"\r\n" );
	  
  return n;
}

//******************************************************************
static uint8_t msg_print_raw( char *str, uint8_t raw, uint8_t i ) {
  uint8_t n = 0;

  if( !i ) n = sprintf( str,"# ");  
  n = sprintf( str+n,"%02x.",raw ); 

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
  case S_SIGNATURE:	// Don't print signature, use state for RSSI
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
    if( msg->error ){
      if( msg->count < msg->nBytes ) {
        nBytes = msg_print_raw( buff, msg->raw[ msg->count ], msg->count );
		msg->count++;
      } else if( msg->nBytes ) {
        nBytes = sprintf( buff, "\r\n" );
		msg->error = 0;
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

static uint8_t msg_print( struct message *msg ) {
   static char msg_buff[TXBUF];
   
  if( msg->state == S_SIGNATURE ) {
    msg->decoded = 0;
	msg->count = 0;
  }

  // Do we still have outstanding text to send?
  if( msg->decoded ) {
    msg->decoded -= tty_put_str( (uint8_t *)msg_buff, msg->decoded );
  }
  
  if( !msg->decoded ) {
    msg->decoded = msg_print_field( msg, msg_buff );
  }
  return msg->decoded;
}

/********************************************************
** RX Message processing
********************************************************/

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
      case S_HEADER:	msgRx->state = msg_rx_header( msgRx, msgRx->decoded ); break;
      case S_ADDR0:     if( msgRx->fields & F_ADDR0 ) { msgRx->state = msg_rx_addr( msgRx, 0, msgRx->decoded ); break; }   /* fallthrough */
      case S_ADDR1:     if( msgRx->fields & F_ADDR1 ) { msgRx->state = msg_rx_addr( msgRx, 1, msgRx->decoded ); break; }   /* fallthrough */
      case S_ADDR2:     if( msgRx->fields & F_ADDR2 ) { msgRx->state = msg_rx_addr( msgRx, 2, msgRx->decoded ); break; }   /* fallthrough */
      case S_PARAM0:    if( msgRx->fields & F_PARAM0 ) { msgRx->state = msg_rx_param( msgRx, 0, msgRx->decoded ); break; } /* fallthrough */
      case S_PARAM1:    if( msgRx->fields & F_PARAM1 ) { msgRx->state = msg_rx_param( msgRx, 1, msgRx->decoded ); break; } /* fallthrough */
      case S_OPCODE:    msgRx->state = msg_rx_opcode( msgRx, msgRx->decoded ); break;
      case S_LEN:       msgRx->state = msg_rx_len( msgRx, msgRx->decoded ); break;
      case S_PAYLOAD:   msgRx->state = msg_rx_payload( msgRx, msgRx->decoded ); break;
      case S_CHECKSUM:  msgRx->state = msg_rx_checksum( msgRx, msgRx->decoded ); break;
	  }
	  msgRx->decoded	 = 0;
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

  msg_rx_ready( &msgRx );
}


void msg_rx_byte( uint8_t byte ) {
  DEBUG_MSG(1);

  if( byte==MSG_START ) {
    msg_rx_start();
  }	else if( msgRx ) {
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

/********************************************************
** TX Message processing
********************************************************/

static uint8_t msg_scan( struct message *msg, uint8_t byte) {
  static char str[17];
  static uint8_t nChar;
  
  if( byte=='\r' || byte=='\n' ) {
	// Ignore blank line
    if( msg->state==S_SIGNATURE && nChar==0 )
      return 0;
	
	// Didn't get a sensible message
    if( msg->state != S_CHECKSUM ) {
	  nChar = 0;
      msg_reset( msg ); // Discard
      return 0;
	} else {
      byte = '\0';
	}
  }
  
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
  str[ nChar++ ] = (char)byte;

  // No spaces between PAYLOAD bytes
  if( byte && msg->state == S_PAYLOAD ) {
    if( nChar==2 ) {
      str[nChar++] = '\0';

      msg_scan_payload( msg, str, nChar );
      msg->nPayload++;
      nChar = 0;

      if( msg->nPayload == msg->len )
        msg->state = S_CHECKSUM;
    } else { // wait for second byte
	  return 0;
	}
  }

  if( !byte ) {
    switch( msg->state ) {
    case S_SIGNATURE:	/* fall through */
	case S_HEADER:      msg_scan_header( msg, str, nChar );	msg->state = S_PARAM0;   break;
	case S_ADDR0:       msg_scan_addr( msg, str, nChar );   msg->state = S_ADDR1;    break;
	case S_ADDR1:       msg_scan_addr( msg, str, nChar );   msg->state = S_ADDR2;    break;
	case S_ADDR2:       msg_scan_addr( msg, str, nChar );   msg->state = S_OPCODE;   break;
    case S_PARAM0:      msg_scan_param( msg, str, nChar );  msg->state = S_ADDR0;    break;
//    case S_PARAM1:
    case S_OPCODE:      msg_scan_opcode( msg, str, nChar ); msg->state = S_LEN;      break;
    case S_LEN:         msg_scan_len( msg, str, nChar );    msg->state = S_PAYLOAD;  break;
    case S_PAYLOAD:                                         msg->state = S_ERROR;    break;
    case S_CHECKSUM:    msg->state = ( nChar>1 ) ? S_ERROR : S_COMPLETE;             break;
//  case S_TRAILER:
//  case S_COMPLETE:
//  case S_ERROR:
    } 
	nChar = 0;
  }

  if( msg->state==S_PAYLOAD ) {
	if( ( msg->rxFields & F_MAND ) != F_MAND ) {
	  msg->state = S_ERROR;
	}
  }

  if( msg->state==S_COMPLETE ) {
    msg->csum += get_header(msg->fields);
	msg->csum = -msg->csum;
	msg->rxFields |= msg->fields;
	return 1;
  }
  
  return 0;
}

static uint8_t msg_tx_process( struct message *msg ) {
  uint8_t byte = 0;
  
  switch( msg->state ) {
  case S_SIGNATURE:	 byte = msg_tx_signature(msg); if( byte )break; msg->state = S_HEADER;   /* fall through */
  case S_HEADER:     byte = msg_tx_header(msg);    if( byte )break; msg->state = S_ADDR0;    /* fall through */
  case S_ADDR0:      byte = msg_tx_addr(msg);      if( byte )break; msg->state = S_ADDR1;    /* fall through */
  case S_ADDR1:      byte = msg_tx_addr(msg);      if( byte )break; msg->state = S_ADDR2;    /* fall through */
  case S_ADDR2:      byte = msg_tx_addr(msg);      if( byte )break; msg->state = S_PARAM0;   /* fall through */
  case S_PARAM0:     byte = msg_tx_param(msg);     if( byte )break; msg->state = S_PARAM1;   /* fall through */
  case S_PARAM1:     byte = msg_tx_param(msg);     if( byte )break; msg->state = S_OPCODE;   /* fall through */
  case S_OPCODE:     byte = msg_tx_opcode(msg);    if( byte )break; msg->state = S_LEN;      /* fall through */
  case S_LEN:        byte = msg_tx_len(msg);       if( byte )break; msg->state = S_PAYLOAD;  /* fall through */
  case S_PAYLOAD:    byte = msg_tx_payload(msg);   if( byte )break; msg->state = S_CHECKSUM; /* fall through */
  case S_CHECKSUM:   byte = msg_tx_checksum(msg);  if( byte )break; msg->state = S_TRAILER;  /* fall through */
  case S_TRAILER:    byte = msg_tx_trailer(msg);   if( byte )break; msg->state = S_COMPLETE; /* fall through */
  case S_COMPLETE:
  case S_ERROR:
    break;
  }
  
  return byte;
}


uint8_t msg_tx_byte( struct message *msg ) {
  uint8_t byte = 0;
  
  if( msg ) {
	byte = msg_tx_process( msg );
  }
  
  return byte;
}

static uint8_t msg_tx_raw( struct message *msg ) {
  static uint8_t n = 0, n1=0, i=0;
  static char buff[4];

  if( n )
	n -= tty_put_str( (uint8_t *)buff, n );
  
  if( !(n+n1) ) {
    uint8_t byte = msg_tx_byte( msg );
	if( byte )
      n = msg_print_raw( buff, byte, i++ );
    else
	  n1 = sprintf( buff, "\r\n" );
  }

  if( n1 )
	n1 -= tty_put_str( (uint8_t *)buff, n1 );
  
  if( !(n+n1) ) i = 0;
  
  return n+n1;
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
  static struct message *rx = NULL;
  static struct message *tx = NULL;
  static struct message *txMsg = NULL;
  
  // Print messages 
  if( rx ) {
    if( !msg_print( rx ) ) {
      msg_free( &rx );
	}
  } else if( txMsg ) {
    if( !msg_tx_raw( txMsg ) ) {
	  msg_tx_ready( &txMsg );
	}
  } else {
	// If we get a message now we'll start printing it next time
	rx = msg_rx_get();
  }
  
  // Scan TX messages
  if( !tx ) {
    tx = msg_alloc();
  }
  
  if( tx ) {
    uint8_t byte = tty_rx_get();
    if( byte ) {
      if( msg_scan( tx, byte ) ) {
//		tx->state = S_SIGNATURE;  txMsg = tx;  tx = NULL;
		msg_tx_ready( &tx );
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

void msg_init( uint8_t class, uint32_t id ) {
  msg_create_pool();

  myClass = class;
  myId = id;
}


