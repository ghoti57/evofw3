#include "config.h"

#if !defined(USE_FIFO)

#include <string.h>
#include "driver.h"
#include "ringbuf.h"
#include "led.h"
#include "errors.h"

static inbound_byte_fn inbound_byte;

// Bytes in this buffer are raw "over the air", and still in Manchester code.
// This allows us to use Manchester-breaking values as signal flags
#define RX_START_OF_FRAME     0
#define RX_FRAME_ERROR_START  1
#define RX_FRAME_ERROR_STOP   2
#define RX_END_OF_FRAME       3
static rb_t rx_buffer;

// Bytes waiting to go to the radio for transmission
static rb_t tx_buffer;

// Transmit states
#define TX_READY   0
#define TX_SENDING 1

// Receive states
#define RX_READY   0
#define RX_MANCH0  1  // Decoding first byte of Manchester pair
#define RX_MANCH1  2  // Decoding second byte of Manchester pair

// Last 3 bytes of the preamble (0x33 0x55 0x53), expressed as a bit-stream with
// their start and stop bits included. Total of 30 bits (3 x (8 + 1 + 1))
#define SYNC_PATTERN       (uint32_t)0x19955595
#define SYNC_PATTERN_LEN   30
#define SYNC_PATTERN_MASK  ((1UL << SYNC_PATTERN_LEN) - 1)

static uint8_t MANCH_ENC[16] = {0xAA, 0xA9, 0xA6, 0xA5, 0x9A, 0x99, 0x96, 0x95,
                                0x6A, 0x69, 0x66, 0x65, 0x5A, 0x59, 0x56, 0x55};
static uint8_t MANCH_DEC[256];

static volatile uint8_t rx_in_sync = 0;
static volatile uint8_t tx_working = 0;

void driver_init(inbound_byte_fn in) {
  inbound_byte = in;

  // Build Manchester decoding table from encoding table
  memset(MANCH_DEC, 0xFF, sizeof(MANCH_DEC));
  for (uint8_t i = 0; i < sizeof(MANCH_ENC); i++) {
    MANCH_DEC[MANCH_ENC[i]] = i;
  }

  rb_reset(&rx_buffer);
  rb_reset(&tx_buffer);
}

void driver_work(void) {
  static uint8_t state = RX_READY;
  static uint8_t manch_byte;

  while (rx_buffer.nbytes) {
    uint8_t b = rb_get(&rx_buffer);
    if (state == RX_READY) {
      if (b == RX_START_OF_FRAME) {
        state = RX_MANCH0;
      }
      break;
    }

    if (b == RX_FRAME_ERROR_START) {
      inbound_byte(0, ERR_BAD_START_BIT);
      state = RX_READY;
      break;
    }

    if (b == RX_FRAME_ERROR_STOP) {
      inbound_byte(0, ERR_BAD_STOP_BIT);
      state = RX_READY;
      break;
    }

    if (b == RX_START_OF_FRAME) {
      inbound_byte(0, ERR_UNEXPECTED_START_OF_FRAME);
      state = RX_READY;
      break;
    }

    if (b == RX_END_OF_FRAME) {
      if (state == RX_MANCH0) {
        // Good EOF
        inbound_byte(0, ERR_NONE);
      } else {
        // Bad EOF
        inbound_byte(0, ERR_UNEXPECTED_END_OF_FRAME);
      }

      state = RX_READY;
      break;
    }

    uint8_t m = MANCH_DEC[b];
    if (m == 0xFF) {
      // Bad Manchester encoding
      inbound_byte(0, ERR_MANCHESTER_ENCODING);
      state = RX_READY;
      break;
    } else if (state == RX_MANCH0) {
      manch_byte = m << 4;
      state = RX_MANCH1;
    } else {
      manch_byte |= m;
      inbound_byte(manch_byte, 0);
      state = RX_MANCH0;
    }
  }
}

void driver_send_byte(uint8_t b, uint8_t end) {
  static uint8_t state = TX_READY;

  if (state == TX_READY) {
    // Preamble, if not already sent this frame
    rb_put(&tx_buffer, 0x55);
    rb_put(&tx_buffer, 0x55);
    rb_put(&tx_buffer, 0x55);
    rb_put(&tx_buffer, 0x55);
    rb_put(&tx_buffer, 0x55);
    rb_put(&tx_buffer, 0xFF);
    rb_put(&tx_buffer, 0x00);
    rb_put(&tx_buffer, 0x33);
    rb_put(&tx_buffer, 0x55);
    rb_put(&tx_buffer, 0x53);
    state = TX_SENDING;
  }

  // Byte, Manchester encoded
  rb_put(&tx_buffer, MANCH_ENC[(b >> 4) & 0x0F]);
  rb_put(&tx_buffer, MANCH_ENC[b & 0x0F]);

  if (end) {
    // Postamble, if end of frame
    rb_put(&tx_buffer, 0x35);
    rb_put(&tx_buffer, 0x55);
    rb_put(&tx_buffer, 0x55);
    rb_put(&tx_buffer, 0x55);
    rb_put(&tx_buffer, 0x55);
    state = TX_READY;
  }
}

uint8_t driver_accept_bit(uint8_t bit) {
  static uint32_t sync_buffer = 0;
  static uint8_t bit_counter = 0;
  static uint8_t byte_buffer = 0;

  sync_buffer <<= 1;
  sync_buffer |= bit;

  if (!rx_in_sync) {
    if (tx_buffer.nbytes) {
      // Not in sync (i.e. not receiving a packet) but something waiting to send
      tx_working = 0;
      return 1;
    }

    if ((sync_buffer & SYNC_PATTERN_MASK) == SYNC_PATTERN) {
      rx_in_sync = 1;
      bit_counter = 0;
      byte_buffer = 0;
      rb_put(&rx_buffer, RX_START_OF_FRAME);
      led_toggle();
    }
    return 0;
  }

  if (bit_counter == 0) {  // start bit
    if (bit) {
      // start bit shouldn't be high
      rb_put(&rx_buffer, RX_FRAME_ERROR_START);
      rx_in_sync = 0;
      return 0;
    }
    bit_counter++;
    return 0;
  }

  if (bit_counter == 9) {  // stop bit
    if (!bit) {
      // stop bit shouldn't be low
      rb_put(&rx_buffer, RX_FRAME_ERROR_STOP);
      rx_in_sync = 0;
      return 0;
    }

    if (byte_buffer == 0x35) {
      rb_put(&rx_buffer, RX_END_OF_FRAME);
      rx_in_sync = 0;
      return 0;
    }

    rb_put(&rx_buffer, byte_buffer);

    // ready for a new bit in a new byte
    bit_counter = 0;
    byte_buffer = 0;
    return 0;
  }

  // bits arrive LSB first - rotate in from the left
  byte_buffer >>= 1;
  if (bit) byte_buffer |= 0x80;
  bit_counter++;
  return 0;
}

uint8_t driver_request_bit(void) {
  static uint8_t bit_counter = 0;
  static uint8_t byte_buffer = 0;

  if (!tx_working) {
    if (!tx_buffer.nbytes) {
      // Nothing buffered to send, and not mid-tx, switch back to listening
      rx_in_sync = 0;
      return 0xFF;
    }
    byte_buffer = rb_get(&tx_buffer);
    bit_counter = 0;
    tx_working = 1;

    // start bit
    bit_counter++;
    return 0;
  }

  if (bit_counter == 9) {  // stop bit
    tx_working = 0;
    return 1;
  }

  uint8_t bit = byte_buffer & 0x01;
  byte_buffer >>= 1;
  bit_counter++;
  return bit;
}

#endif // !USE_FIFO
