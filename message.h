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

struct nmessage;
extern struct message *msg_tx_get(void);
extern uint8_t msg_tx_byte( struct message *msg );
extern void msg_tx_done( struct message **msg );

extern void msg_rx_byte(uint8_t byte);
extern void msg_rx_rssi( uint8_t rssi );

extern void msg_init(uint8_t class, uint32_t id );
extern void msg_work(void);

#endif // _MESSAGE_H_
