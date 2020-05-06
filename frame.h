#ifndef _FRAME_H_
#define _FRAME_H_

// UART interface
#define FRM_START     0xF0
#define FRM_LOST_SYNC 0xF1
#define FRM_END       0xFF
extern void frame_rx_byte(uint8_t byte);

extern void frame_rx_enable(void);
extern void frame_tx_enable(void);
extern void frame_disable(void);

extern void frame_init(void);
extern void frame_work(void);

#endif // _FRAME_H_
