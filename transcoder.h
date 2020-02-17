#ifndef __TRANSCODER_H__
#define __TRANSCODER_H__

#include <stdint.h>

typedef void (*write_str_fn)(char*);
typedef void (*send_byte_fn)(uint8_t, uint8_t end);
void transcoder_init(write_str_fn, send_byte_fn);
void transcoder_work(void);

void transcoder_accept_inbound_byte(uint8_t b, uint8_t status);
void transcoder_accept_outbound_byte(uint8_t b);

extern uint16_t transcoder_param_len( uint8_t hdr  );
extern void transcoder_rx_byte( uint8_t byte );

enum transcoder_rx_status {
  TC_RX_IDLE,
  TC_RX_START,
  TC_RX_RSSI,
  TC_RX_END,
  TC_RX_ABORTED,
  TC_RX_MANCHESTER_DECODE_ERROR,
  TC_RX_CHECKSUM_ERROR
};
extern void transcoder_rx_status( uint8_t status );


#endif
