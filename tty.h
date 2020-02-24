/**************************************************************************
** tty.h
** 
** Host facing UART 
*/
#ifndef _TTY_H_
#define _TTY_H_

#include <stdint.h>

#define TXBUF 32
#define RXBUF 32

// Application UART write
//   nByte MUST be less than TXBUF
//   returns number of bytes sent to UART
extern uint8_t tty_put_str( uint8_t *byte, uint8_t nByte );
extern void tty_start_tx(void);
extern void tty_stop_tx(void);

extern uint8_t tty_rx_get(void);
extern void tty_start_rx(void);
extern void tty_stop_rx(void);

extern void tty_init(void);
extern void tty_work(void);

#endif // _TTY_H_
