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

#ifdef LOG_TIME
extern void msg_rx_byte(uint8_t byte, uint8_t time);
#else
extern void msg_rx_byte(uint8_t byte);
#endif
extern void msg_rx_rssi( uint8_t rssi );

extern void msg_init(void);
extern void msg_work(void);

#endif // _MESSAGE_H_
