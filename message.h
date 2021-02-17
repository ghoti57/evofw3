/********************************************************
** message.h
**
** Packet conversion to message
**
********************************************************/
#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#define _MSG_ERR_LIST \
  _MSG_ERR( MSG_SIG_ERR,      "Bad Signature" ) \
  _MSG_ERR( MSG_SYNC_ERR,     "Lost Sync" ) \
  _MSG_ERR( MSG_CLSN_ERR,     "Collision" ) \
  _MSG_ERR( MSG_MANC_ERR,     "Invalid Manchester Code" ) \
  _MSG_ERR( MSG_OVERRUN_ERR,  "Too long" ) \
  _MSG_ERR( MSG_CSUM_ERR,     "Checksum error" ) \
  _MSG_ERR( MSG_TRUNC_ERR,    "Truncated" ) \
  _MSG_ERR( MSG_WARNING,      "Warning" ) \
  _MSG_ERR( MSG_SUSPECT_WARN, "Suspect payload" ) \
  _MSG_ERR( MSG_BAD_TX,       "Bad TX message" ) \

#define _MSG_ERR(_e,_t) _e,
enum msg_err_code { MSG_OK=0, _MSG_ERR_LIST MSG_ERR_MAX };
#undef _MSG_ERR

/******************************************
** Frame interface
*/
extern uint8_t *msg_rx_start(void);
extern uint8_t msg_rx_byte(uint8_t byte);
extern void msg_rx_end( uint8_t nBytes, uint8_t error );
extern void msg_rx_rssi( uint8_t rssi );

extern uint8_t msg_tx_byte(uint8_t *done);
extern void msg_tx_end( uint8_t nBytes );
extern void msg_tx_done(void);

/******************************************
** Application interface
*/
struct message;
extern void msg_free( struct message **msg );
extern struct message *msg_alloc(void);

extern uint8_t msg_isValid( struct message *msg );
extern uint8_t msg_isTx( struct message *msg );

// RX messages
extern void msg_rx_ready( struct message **msg );
extern struct message *msg_rx_get(void);
extern uint8_t msg_print( struct message *msg, char *msgBuff );
extern void msg_change_addr( struct message *msg, uint8_t addr, uint8_t Class,uint32_t Id , uint8_t myClass,uint32_t myId );

// TX Messages
extern void msg_tx_ready( struct message **msg );
extern uint8_t msg_scan( struct message *msg, uint8_t byte );
extern void msg_change_addr( struct message *msg,uint8_t addr, uint8_t id,uint32_t class , uint8_t myId,uint32_t myClass );  

extern void msg_init(void);
extern void msg_work(void);

#endif // _MESSAGE_H_
