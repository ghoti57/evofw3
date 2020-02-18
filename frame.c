#include <string.h>
#include <util/delay.h>

#include <avr/interrupt.h>

#include "config.h"
#include "cc1101.h"
#include "message.h"
#include "frame.h"

#define DEBUG_ISR(_v)      DEBUG1(_v)
#define DEBUG_EDGE(_v)     DEBUG2(_v)
#define DEBUG_FRAME(_v)    DEBUG3(_v)

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
#define STOP_BITS_MAX   ( 12 * ONE_BIT + HALF_BIT )

/***********************************************************************************
** RX Frame state machine
*/

enum rx_states {
  RX_OFF,
  RX_IDLE,		// Make sure we've seen an edge for valid interval calculations
  RX_PREAMBLE0, // Look for a ZERO bit
  RX_PREAMBLE1, // Look for a ONE bit
  RX_SYNC0,		// Check for SYNC0 (0xFF) - revert to RX_START if not found
  RX_SYNC1,		// Check for SYNC1 (0x00) - revert to RX_START if not found
  RX_STOP,		// Check for STOP bit - revert to RX_START if not found
  RX_FRAME0,	// First edge in byte within frame
  RX_FRAME,		// Rest of byte
  RX_DONE		// End of frame reached - discard everything
};

static struct rx_state {
  uint16_t time;
  uint16_t lastTime;
  uint16_t time0;

  uint8_t level;
  uint8_t lastLevel;

  uint8_t state;
  uint8_t preamble;
  
  uint8_t nByte;
  uint8_t lastByte;

  // Edge buffers
  uint8_t Edges[2][24];
  uint8_t NEdges[2];
  
  // Current edges
  uint8_t idx;
  uint8_t nEdges;
  uint8_t *edges;
} rx;

static void rx_reset(void) {
  memset( &rx, 0, sizeof(rx) );
  rx.edges = rx.Edges[ rx.idx ];
}

static void rx_frame_start(void);
static void rx_frame_end(void);
static void rx_frame_done(void);

static void rx_stop(void);
static void rx_start(void);

/***********************************************************************************
** FRAME detection
**
** A frame begins with ...0101s<FF>ps<00>p
** since s=0 and p=1 we can just look for the following pattern
** 1x1 1x0 9x1 9x0 1x1
**
** If the pattern fails at any point normally go back to the corresponding
** 1x0 or 1x1
**
** This is a very lightweight approach to detecting start of frame
** when there's likely to be a lot of noise 
*/

//-----------------------------------------------------------------------------
// RX reset assumes last edge was falling edge 
// Make sure we've seen a rising edge before we do interval measurement 
static uint8_t rx_idle( void) {
  uint8_t state = RX_IDLE;

  if( rx.level )
    state = RX_PREAMBLE0;
	
  return state;
}

//-----------------------------------------------------------------------------
// look for single 0 bit
static uint8_t rx_preamble0( uint8_t interval ) {
  uint8_t state = RX_PREAMBLE0;		// Stay here until we see a ZERO

  if( rx.level ) { // rising edge - was previous LOW correct length?
    if( interval >= MIN_BIT && interval <= MAX_BIT ) {
	  rx.preamble++;
      state = RX_SYNC0;
    } else {
	  rx.preamble = 0;
	  state = RX_PREAMBLE1;
	}
  }
  
  return state;
}

//-----------------------------------------------------------------------------
// look for single 1 bit
static uint8_t rx_preamble1( uint8_t interval ) {
  uint8_t state = RX_PREAMBLE1;		// Stay here until we see a ONE

  if( !rx.level ) { // falling edge - was previous HIGH correct length?
    if( interval >= MIN_BIT && interval <= MAX_BIT ) {
	  rx.preamble++;
	} else {
	  rx.preamble = 0;
	}
    state = RX_PREAMBLE0;
  } else {
    rx.preamble = 0;
  }

  return state;
}

//-----------------------------------------------------------------------------
// look for nine succesive 1s
static uint8_t rx_sync0( uint8_t interval ) {
  uint8_t state = RX_IDLE;	// Will never actually go here

  if( !rx.level ) { // falling edge - was previous HIGH correct length?
    if( interval >= NINE_BITS_MIN && interval <= NINE_BITS_MAX ) {
	  rx.preamble = 0;
	  state = RX_SYNC1;
	} else {
	  state = rx_preamble1( interval );
	}
  } else {
	rx.preamble = 0;
    state = rx_preamble0( interval );
  }
  
  return state;
}

//-----------------------------------------------------------------------------
// look for nine succesive 0s
static uint8_t rx_sync1( uint8_t interval ) {
  uint8_t state = RX_IDLE;	// Will never actually go here

  if( rx.level ) {  // rising edge - was previous LOW correct length?
    if( interval >= NINE_BITS_MIN && interval <= NINE_BITS_MAX ) {
	  state = RX_STOP;
    } else {
      state = rx_preamble0( interval );
	} 
  } else {
    state = rx_preamble1( interval );
  }

  return state;
}

//-----------------------------------------------------------------------------
// look for single 1 bit
static uint8_t rx_stop_bit( uint8_t interval ) {
  uint8_t state = RX_IDLE;	// Will never actually go here

  if( !rx.level ) {  // falling edge - was previous HIGH correct length?
    if( interval >= MIN_BIT && interval <= MAX_BIT ) {
	  state = RX_FRAME0;
	  rx_frame_start();
    } else {
	  state = rx_preamble1( interval );
    }
  } else {
    state = rx_preamble0( interval );
  }

  return state;
}


/***************************************************************************
** RX frame processing
*/

static void rx_frame_start(void) {
  DEBUG_FRAME(1);
  msg_rx_byte(MSG_START);
}

static void rx_frame_end(void) {
  DEBUG_FRAME(0);
  rx_stop();
}

static void rx_frame_done(void) {
  uint8_t rssi = cc_read_rssi();
  msg_rx_rssi( rssi );
  msg_rx_byte(MSG_END);
};

// Callback from edge processing
void msg_last_byte(uint8_t byte) {
  rx.lastByte = byte;
}

static void rx_byte(void) {
  rx.nByte++;
  
  // Switch edge buffer
  rx.NEdges[rx.idx] = rx.nEdges;
  rx.idx ^= 1;
  rx.edges = rx.Edges[rx.idx];
  rx.nEdges = 0;

  SW_INT_PIN |= SW_INT_IN;
}

static uint8_t rx_frame(uint8_t interval) {
  uint8_t state = RX_FRAME;

  rx.edges[rx.nEdges++] = interval;
  
  if( interval>TEN_BITS_MIN ) {
    if( interval < STOP_BITS_MAX ) { // Possible stop bit
	  if( !rx.level ) { // Was a falling edge so probably valid stop bit
	    rx_byte();
		state = RX_FRAME0;
	  } else { // Lost BYTE synch 
        rx_frame_end();
        state = RX_DONE;
	  }
    } else { // lost BYTE synch
      rx_frame_end();
      state = RX_DONE;
    }
  } else if( rx.lastByte==0xAC ) {
    rx_frame_end();
    state = RX_DONE;
  }
  
  return state;
}

/***************************************************************************
** RX edge processing
*/


static uint8_t rx_edge(uint8_t interval) {
  uint8_t synch = 1;

  switch( rx.state ) {
  case RX_IDLE:       rx.state = rx_idle();               break;

  // Frame detect states
  case RX_PREAMBLE0:  rx.state = rx_preamble0(interval);  break;
  case RX_PREAMBLE1:  rx.state = rx_preamble1(interval);  break;
  case RX_SYNC0:      rx.state = rx_sync0(interval);      break;
  case RX_SYNC1:      rx.state = rx_sync1(interval);      break;
  case RX_STOP:       rx.state = rx_stop_bit(interval);   break;

  // Frame processing
  case RX_FRAME0:     // FRAME0 Only used to signal clock recovery
  case RX_FRAME:      rx.state = rx_frame(interval);      break;
  }

  // When we're in a frame mode only synch time0 at the end of bytes
  // This allows us to perform clock recovery on the stop/start bit
  // boundary.
  if( rx.state == RX_FRAME )
    synch = 0;
  
  return synch;
}


/***********************************************************************************
** RX
** On Edge interrupts from the radio signal use the counter as a timer
**
** The difference between the counts on 2 successive edges gives the width
** of a LOW or HIGH period in the signal immediately before the latest edge
**
*/

#define RX_CLOCK TCNT1

static uint8_t clockShift;

ISR(GDO0_INT_VECT) {
  DEBUG_ISR(1);

  rx.time  = RX_CLOCK;                // Grab a copy of the counter ASAP for accuracy
  rx.level = ( GDO0_PIN & GDO0_IN );  // and the current level

  if( rx.level != rx.lastLevel ) {
    uint8_t interval = ( rx.time - rx.time0 ) >> clockShift;

	uint8_t synch = rx_edge( interval );
	if( synch ) rx.time0 = rx.time;

    rx.lastLevel = rx.level;
    rx.lastTime  = rx.time;
  }

  DEBUG_ISR(0);
}


/***************************************************************************
** Enable a free-running counter that gives us a time reference for RX
*/

static void rx_init(void) {
  uint8_t sreg = SREG;
  cli();

  TCCR1A = 0; // Normal mode, no output pins

  // We want to prescale the hardware timer as much as possible
  // to maximise the period between overruns but remain above 500 KHz
  TCCR1B = ( 1<<CS11 ); // Pre-scale by 8

  // This is the additional scaling required in software to reduce the 
  // clock rate to 500 KHz
  clockShift = ( F_CPU==16000000 ) ? 2 : 1;

  SREG = sreg;
}

/********************************************************
** Edge analysis ISR
** In order to avoid delaying the measurement of edges
** the analysis of the edges is run in a lowewr priority
** ISR.
********************************************************/

static uint8_t rx_process_edges( uint8_t *edges, uint8_t nEdges ) {
  uint8_t rx_byte = 0;
  uint8_t rx_t = 0;
  uint8_t rx_tBit = ONE_BIT;
  uint8_t rx_isHi = 0;
  uint8_t rx_hi = 0;

  DEBUG_EDGE( 1 );

  while( nEdges-- ) {

	uint8_t interval = *(edges++);
    if( rx_tBit < TEN_BITS ) { // 
      uint8_t samples = interval - rx_t;
      while( samples ) {
        uint8_t tBit = rx_tBit - rx_t;
        if( tBit > samples )
          tBit = samples;
  
        if( rx_isHi ) rx_hi += tBit;

        rx_t += tBit;
        samples -= tBit;

        // BIT complete?	  
        if( rx_t==rx_tBit ) {
		  if( rx_tBit == ONE_BIT ) { // START BIT
		  }
          else if( rx_tBit < TEN_BITS ) {  
            uint8_t bit = ( rx_hi > HALF_BIT );
            rx_byte <<= 1;
            rx_byte  |= bit;
          } 

          rx_tBit += ONE_BIT;
          rx_hi = 0;
        }
      }
    }

	// Edges toggle level
    rx_isHi ^= 1;
  }

  DEBUG_EDGE( 0 );

  return rx_byte;
}

ISR(SW_INT_VECT) {
  // Very important that we don't block interrupts
  // As this interferes with subsequent edge measurements
  sei();

  // Extract byte from previous edges
  rx.lastByte = rx_process_edges( rx.Edges[1-rx.idx], rx.NEdges[1-rx.idx] );
  
  // And pass it on to message to process
  msg_rx_byte( rx.lastByte );
}


//---------------------------------------------------------------------------------

static void rx_start(void) {
  uint8_t sreg = SREG;
  cli();

  // Make sure configured as input in case shared with TX
  GDO0_DDR  &= ~GDO0_IN;
  GDO0_PORT |=  GDO0_IN;		 // Set input pull-up
  
  EICRA |= ( 1 << GDO0_INT_ISCn0 );   // rising and falling edge
  EIFR   = GDO0_INT_MASK ;     // Acknowledge any previous edges
  EIMSK |= GDO0_INT_MASK ;     // Enable interrupts
  
  // Configure SW interrupt for edge processing
  SW_INT_DDR  |= SW_INT_IN;
  SW_INT_MASK |= SW_INT_IN;

  PCIFR  = SW_INT_ENBL;	// Acknowledge any previous event
  PCICR |= SW_INT_ENBL;	// and enable
  
  SREG = sreg;

  // TODO: put the radio in RX mode
}

//---------------------------------------------------------------------------------

static void rx_stop(void) {
  uint8_t sreg = SREG;
  cli();

  EIMSK &= ~GDO0_INT_MASK;                 // Disable interrupts
  
  SREG = sreg;
  
  // TODO: put the radio in IDLE mode
}


/***************************************************************************
** External interface
*/

void frame_rx_enable(void) {
  uint8_t sreg = SREG;
  cli();
  
  rx_reset();
  rx.state = RX_IDLE;

  SREG = sreg;
    
  rx_start();
  // TODO: radio to RX mode
}

void frame_rx_disable(void) {
  rx_stop();
  // TODO: radio to IDLE mode
}

void frame_init(void) {
  rx_reset();
  rx_init();
}

void frame_work(void) {
  if( rx.state==RX_DONE ) {
	rx_frame_done();
    frame_rx_enable();
  }
  if( rx.state==RX_OFF ) {
    frame_rx_enable();
  }
  
}
