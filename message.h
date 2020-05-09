/********************************************************
** message.h
**
** Packet conversion to message
**
********************************************************/
#ifndef _MESSAGE_H_
#define _MESSAGE_H_

enum message_flags {
  MSG_START = 0xF0,
  MSG_END
};

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

#define _MSG_ERR(_e,_t) _e,
enum msg_err_code { MSG_OK=0, _MSG_ERR_LIST MSG_ERR_MAX };
#undef _MSG_ERR

extern uint8_t *msg_rx_start(void);
extern uint8_t msg_rx_byte(uint8_t byte);
extern void msg_rx_end( uint8_t nBytes, uint8_t error );

extern void msg_rx_rssi( uint8_t rssi );

extern void msg_init(uint8_t myClass, uint32_t myID );
extern void msg_work(void);

#endif // _MESSAGE_H_
