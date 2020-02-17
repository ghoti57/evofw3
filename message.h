/********************************************************
** message.h
**
** Packet conversion to message
**
********************************************************/
#ifndef _MESSAGE_H_
#define _MESSAGE_H_

/***************************************************************
** BIT constants
** These are based on a 500 KHz clock
** 500000/38400 is almost exactly 13
**
** Clock rates that are multiples of 500 KHz cause these
** constants to increase such that they do not all fit 
** in uint8_t variables.
**
** Maintaining the variables as uint8_t significantly improves
** the efficiency of the RX processing code.
*/
#define BAUD_RATE 38400

#define ONE_BIT  13
#define HALF_BIT 6
#define BIT_TOL  4

#define MIN_BIT  ( ONE_BIT - BIT_TOL )
#define MAX_BIT  ( ONE_BIT + BIT_TOL )

#define NINE_BITS		( 9 * ONE_BIT )
#define NINE_BITS_MIN	( NINE_BITS - HALF_BIT )
#define NINE_BITS_MAX	( NINE_BITS + HALF_BIT )

#define TEN_BITS		( 10 * ONE_BIT )
#define TEN_BITS_MIN	( TEN_BITS - HALF_BIT )
#define TEN_BITS_MAX	( TEN_BITS + HALF_BIT )

enum message_flags {
  MSG_START = 0xF0,
  MSG_END
};

extern void msg_rx_byte(uint8_t byte);
extern void msg_rx_edges( uint8_t *edges, uint8_t nEdges );
extern void msg_rx_rssi( uint8_t rssi );

// Frame callback
extern void msg_last_byte(uint8_t byte);

extern void msg_init(void);
extern void msg_work(void);

#endif // _MESSAGE_H_
