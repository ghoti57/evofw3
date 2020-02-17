#include <stdint.h>

#include "config.h"

#include "ringbuf.h"
#include "cc1101.h"
#include "transcoder.h"
#include "bitstream.h"

/*********************************************************
*
* bitstream.c
* ===========
*
* Provide an interface beween the radio interface (cc1101)
* and transcode.c
*
* CC1101 produces and consumes bitstreams in the form of octets.
*
* The bitstreams are actually UART bitstreams that include
* start/stop bits.  The CC1101 produces/consumes 5 octets
* for every 4 UART bytes.
*
* The description that follows describes the bit stream as
* seen by the radio and in paricular the bit order seen there
*
* The structure of a bitstream packet is
*
* [bit synch stream][Evo Header][Evo Message][Evo Trailer][bit synch steam]
*
* The [bit synch stream] is a continuos stream of alternating 0 and 1
* i.e. an arbitrary length string of 0101010101...
* This can be represented as a string of (5) 0xAA bytes
* complete with start(s) and stop(p) bits
*   <                   50 bits                      >
*   01010101010101010101010101010101010101010101010101
*   s<  AA  >ps<  AA  >ps<  AA  >ps<  AA  >ps<  AA  >p
*
* The Evo header is a constant fixed length string
*   <                   50 bits                      >
*   01111111110000000001011001100101010101010110010101
*   s<  FF  >ps<  00  >ps<  CC  >ps<  AA  >ps<  CA  >p
*
* The Evo trailer is a single byte value
*   < 10 bits>
*   0101011001
*   s<  AC  >p
*
* The Evo message is a manchester encoded sequence where each
* byte of the actual message is generated from two bytes of
* data from the bitstream.
*   <      20 bits     >
*   0MMMMMMMM10mmmmmmmm1
*   s<  MSB >ps< lsb  >p
*
* The manchester codes are inserted into the bitstream as little-endian
* octets
*
* The description above is in terms of UART bytes including the start/stop bits
* The CC1101 technical documentation describes the bitream as pure octet
* streams where the start/stop bits are just part of the stream.
*
* There are a couple of important observations concerning this when using the
* hardware packet handler.
*
* The bit synchronisation preamble is described as an alternating sequence
* of 1 and 0, the opposite of the description above.
*
* In TX mode the CC1101 will automatically transmit the specified
* number of 10101010 premable octets followed by the 16 bit SYNC1/0 value.
* Everything after that will be the bit stream specified by the data
* supplied in the FIFO
*
*
*************************************************************************
*
* Radio synchronisation of received bytes
*
* The CC1101 is configured to recognise the start of a packet by
* scanning for a [bit synch pattern] to generate the bit clock
* and then matching a 16 bit SYNC word providing octet alignment
* for the following bits.
*
* The CC1101 strips the [bit synch pattern] and SYNC word then starts
* delivering synchronised octets to bitstream via bs_accept_octet
*
* By careful selection of the value used for the CC1101 SYNC word
* we can optimise the behaviour for bs_accept_octet
*
* The first octets to be processed by bs_accept_octet can be treated as
* a further synchronisation pattern EVO_SYNCH
*
* The SYNC word should not be separated from the training bits
* by any repeated 0s or 1s and EVO_SYNCH should terminate without
* leaving any bits that cannot easily be considered as part of a data byte
*
* The interpretation of the bit stream at the beginning of each packet and
* the proposed values for SYNC1/0 and EVO_SYNCH are
*
*               ><                 50 bits                        ><
*      0101010101011111111100000000010110011001010101010101100101010xxxxxxxx10xxxxxxxx1
*      s<  AA  >ps<  FF  >ps<  00  >ps<  CC  >ps<  AA  >ps<  CA  >ps<  MM  >ps<  mm  >p
*   N x 10101010 ><  SYNC1/0     ><          EVO_SYNCH           >< Evo message
*                ><  16 bits     ><          32 bits             ><   8  ><  8   >< ...
*      preamble  ><  0xFF 0x80   ><     0x2C 0xCA 0xAA 0xCA      >
*
*  NOTES:
*   SYNC begins with what is the first data bit of the FF byte
*        the corresponding start bit is left as part of [bit synch pattern]
*   EVO_SYNCH is the 32 bits that immediately follow SYNC1/0
*             including the last 2 data bits of 0x00 but
*             excluding the stop bit of 0xCA
*             32 bits means that exactly 4 octets are occupied
*   The first octet following EVO_SYNCH will be of the form psBBBBBB
*
*   These values are consistent with the required TX behaviour
*/

#define CC1101_SYNC 0xFF80     /* 1111 1111 1000 0000 */
#define EVO_SYNCH   0x2CCAAACA /* 0010 1100 1100 1010 1010 1010 1100 1010 */
#define EVO_EOF     0xAC       /* 1010 1100 */
#define BIT_TRAIN   0xAA       /* 1010 1010 */

static uint8_t evo_header[4];

uint16_t bs_sync_word(void){ return CC1101_SYNC; }

/*******************************************************
* Manchester Encoding
*
* The [Evo Message] is encoded in the bitstream with a
* Manchester encoding.  This is the only part of the
* complete packet encoded in this way so we cannot
* use the built in function of the CC1101
*
* While the bitstream is interpreted as a Big-endian stream
* the manchester codes inserted in the stream are little-ndian
* 
* The Manchester data here is designed to correspond with the
* Big-endian byte stream seen in the bitstream.
*
********
* NOTE *
********
* The manchester decode process converts the data from 
*     2x8 bit little-endian to 8 bit big-endian
* The manchester encode process converts the data from 
*     8 bit big-endian to 2x8 bit little-endian
*
* Since only a small subset of 8-bit values are actually allowed in
* the bitstream rogue values can be used to identify some errors in 
* the bitstream.
*
*/

static uint8_t const man_encode[16] = {
  0x55, 0x95, 0x65, 0xA5, 0x59, 0x99, 0x69, 0xA9,
  0x56, 0x96, 0x66, 0xA6, 0x5A, 0x9A, 0x6A, 0xAA
};

static uint8_t man_decode[256];

static void manchester_init(void) {
  uint16_t i;
  
  for( i=0; i<sizeof( man_decode ); i++ )
    man_decode[ i ] = 0xFF;

  for( i=0; i<sizeof( man_encode ); i++ )
    man_decode[ man_encode[i] ] = i;
}

static inline int manchester_code_valid( uint8_t code ) { return man_decode[code]!=0xFF ; }

static inline uint8_t manchester_decode( uint8_t byte1, uint8_t byte2 ) { 
  uint8_t decoded;
  
  decoded  = man_decode[byte1] << 4;
  decoded |= man_decode[byte2];

  return decoded;
}

static inline void manchester_encode( uint8_t value, uint8_t *byte1, uint8_t *byte2 ) {
  *byte1 = man_encode[ ( value >> 4 ) & 0xF ];
  *byte2 = man_encode[ ( value      ) & 0xF ];
}

/*****************************************************************
* NOTE: The following shift_register structure is sensitive to
*       the endianness used by the MCU.  It may be necessary to
*       swap the order of .bits and .data
*
* The desired behaviour is that when .reg is left shifted the
* msb of .bits becomes the lsb of .data
*/
union shift_register {
  uint16_t reg;
  struct {
#if __BYTE_ORDER__ ==__ORDER_LITTLE_ENDIAN__
    uint8_t bits;
    uint8_t data;
#endif
#if __BYTE_ORDER__ ==__ORDER_BIG_ENDIAN__
    uint8_t data;
    uint8_t bits;
#endif
  };
};

/*******************************************************
* RX byte processing
*
*
* The overall structure of the Evo Message is
* (header)(params)(cmd)(len)(payload)(chksum)
*
* Processing of the bitstream is only interested in
* this structure in terms of overall validation and in
* providing feedback to CC1101 during message RX
*
* (header) is a single byte that specifies the fields that are present
*          in the (params) field and hence the length of (params)
* (params) is a variable length byte string containing a variety
*          of device addresses and optional parameters associated
*          with the message
* (cmd)    16 bit command value
* (len)    8 bit value specifying the length of the following (payload)
* (payload) variable length byte string associated with the command
* (chksum) 8 bit checksum
*
* Octets received from cc1101 can only be of 10 specific
* types characterised by the location of any start/stop
* bits contained within them:
*  9 sBBBBBBB
*  8 psBBBBBB
*  7 BpsBBBBB
*  6 BBpsBBBB
*  5 BBBpsBBB
*  4 BBBBpsBB
*  3 BBBBBpsB
*  2 BBBBBBps
*  1 BBBBBBBp
*  0 BBBBBBBB
*
* Once EVO_SYNCH has been identified in the bitstream
* subsequent octets received from the cc1101 will rotate
* around either an even or odd 5 octet (40 bit) cycle of
* these pattern to generate 4 manchester encoded bytes
*
* Even aligmnent
* 8 psBBBBBB
* 6 --------BBpsBBBB
* 4 ----------------BBBBpsBB
* 2 ------------------------BBBBBBps
* 0 --------------------------------BBBBBBBB
*
* Odd alignment
* 9  sBBBBBBB
* 7  --------BpsBBBBB
* 5  ----------------BBBpsBBB
* 3  ------------------------BBBBBpsB
* 1  --------------------------------BBBBBBBp
*
* Which cycle and the starting point in the cycle depends on
* the alignment of the Evo Header within the delivered bytes
* If CC1101 is correctly configured this should always be type 8
*
* The synchronised nature of the received bitstream means the
* structure of the next octet to be received is always known
* and hence the processing required for that octet is predicatable.
* We know exactly which bits to keep or discard and when we will
* have a complete manchester encoded byte available
*
* The job of the receive processing is to convert the manchester
* encoded bytes, convert them and send them to transcoder.
*
*/

// RX packet state
static uint16_t rx_pktLen;		// Expected length of packet
static uint16_t rx_octets;		// Number of octets received from cc1101 in this packet
static uint16_t synch_bytes;	// Number of synchronised bytes processed
static uint8_t  rx_checksum;
static uint16_t decoded_bytes;	// Number of decoded message bytes derived from packet
static uint8_t	decode_error;	// current decoded byte contains invalid manchester code
static uint8_t	decoded;		// last decoded byte value
static uint16_t payload_offset;	// location in decoded bytes of message payload

// Start/stop bit stripping
static union shift_register rx;
static uint8_t rxBits;

// Packet synchronisation state
static uint32_t synch_pattern;
static uint8_t  synch_bits;
static uint16_t synchronised;

static void rx_reset(void) {
  rx_pktLen = 0xFFFF;
  rx_octets = 0;
  rx_checksum = 0;
  synch_bytes = 0;
  decoded_bytes = 0;
  decode_error = 0;
  decoded = 0;
  payload_offset = 0xFFFF;
}

static inline void discard( uint8_t n ) { rx.bits <<= n ; }
static inline void keep( uint8_t n ) { rx.reg <<= n ; }
static void rx_data(void) { // Process the byte now in rx.data
  static uint8_t byte1;
  uint8_t byte = rx.data;

  // All bytes received here should be manchester encoded values
  decode_error <<= 4;	// Rotate out old decode errors
  if( !manchester_code_valid( byte ) ) {
    // The one valid exception is the Evo End of message byte
    if( byte == EVO_EOF ) {
      // TODO: is this expected?
      // have we processed the payload and checksum
      decoded = 0;
      return;
    } else {
      decode_error |= 1;
      byte = man_decode[15];	// Protect against subsequent decode - generate 0xF
      transcoder_rx_status(TC_RX_MANCHESTER_DECODE_ERROR);
    }
  }

  synch_bytes++;
  if( synch_bytes & 1 ) {
    // cache first byte of pair
    byte1 = byte;
    return;
  }

  // pass it to transcoder
  decoded = manchester_decode( byte1, byte );
  rx_checksum += decoded;
  decoded_bytes++;

  transcoder_rx_byte( decoded );
}

uint16_t bs_accept_octet( uint8_t bits ) {

  /*------------------------------------------------------------*/
  /* Special values used to control the bitstream processing    */
  /* The values used should never be seen in the received bytes */
  /*------------------------------------------------------------*/
  if( bits==0x00 ) { // start new packet
    if( synchronised && decoded_bytes>0 ) {
      // We've already told transcoder about current packet.
      // better tell it the bad news
      transcoder_rx_status(TC_RX_ABORTED);
    }

    rx_reset();

    synchronised = BS_NOT_SYNCHRONISED;
    synch_bits = 0;

    transcoder_rx_status(TC_RX_START);
    return synchronised;
  }

  if ( bits==0xFF ) { // end of current packet
    if( synchronised && decoded_bytes>0 ) {
      // There is an active packet - make sure it's cleaned up correctly

      if( 1 ) { // TODO: determine condition
        transcoder_rx_status( TC_RX_END );
      } else {
        transcoder_rx_status( TC_RX_ABORTED );
      }
    }

    // Reset fro next packet
    synchronised = BS_NOT_SYNCHRONISED;
    return rx_octets;  // Tell CC1101 how many octets it gave us
  }

  // Real octet
  rx_octets++;

  // TODO: check the CC1101 isn't sending us spurious bytes beyond the end of the packet

  if( synchronised == BS_ABORT ) {
    return synchronised;
  }

  /*------------------------------------------------------------*/
  /* If necessary, find the Evo frame header that synchronises  */
  /* the received bytes                                         */
  /*------------------------------------------------------------*/
  if( !synchronised ) { // we still need to find the Evo header SYNCH_PATTERN
    uint8_t mask = 0x80;
    while( mask ) {
      // Move another bit into the synch register
      synch_pattern <<= 1;
      if( mask & bits ) synch_pattern |= 1;
      mask >>= 1;

      // Check for synch pattern
      if( synch_bits < 32 )
        synch_bits++;
      if( synch_bits==32 && synch_pattern==EVO_SYNCH )
        synchronised = BS_SYNCHRONISED;

      if( synchronised ) { // Found it!
        // Now work out what state the next byte represents
        // Start by assuming we used all the bits of the last octet
        rxBits = 8;
        while( mask ) { // Logically process any remaining bits
          // Cycle the state for each bit we haven't consumed
          // This includes stop/start bits to be discarded
          rxBits = ( rxBits + 1 ) % 10;
          mask >>= 1;
        }

        // Any remaining bits are already correctly aligned for .data
        // Any bits we've processed or logically discarded will
        // be removed by the next rx_data operation (no need to zero them)

        // Initialise the upper half of the shift register
        rx.data = bits ;
      }
    }

    // TODO: Consider error if EVO_SYCH not found in a timely manner

    return synchronised;
  }

  /*----------------------------------------------------*/
  /* If we get here we're processing synchronised bytes */
  /*----------------------------------------------------*/

  // Load the lower half of the shift register
  rx.bits = bits;

  /* Observe that in the following switch the state implies
   * the number of unused bits already in the shift register
   * States 8&9 imply -2 and -1 bits because the first bits
   * of the next octet will be discarded
   *
   * We don't bother to explicitly discard trailing bits in 2,1
   * Every other case explicitly processes 8 bits
   */
  switch( rxBits )
  {
  // Even bit alignment
  case 8:                       discard(2); keep(6); rxBits=6; break;
  case 6: keep(2);  rx_data();  discard(2); keep(4); rxBits=4; break;
  case 4: keep(4);  rx_data();  discard(2); keep(2); rxBits=2; break;
  case 2: keep(6);  rx_data();  /* discard(2); */    rxBits=0; break;
  case 0: keep(8);  rx_data();                       rxBits=8; break;

  // Odd bit alignment
  case 9:                       discard(1); keep(7); rxBits=7; break;
  case 7: keep(1);  rx_data();  discard(2); keep(5); rxBits=5; break;
  case 5: keep(3);  rx_data();  discard(2); keep(3); rxBits=3; break;
  case 3: keep(5);  rx_data();  discard(2); keep(1); rxBits=1; break;
  case 1: keep(7);  rx_data();  /* discard(1); */    rxBits=9; break;
  }

  if( decode_error ) {
    // If we get a manchester decoding error
    //   - keep sending data to transcoder for diagnostic purposes
    //   - continue to process RX bytes from CC1101 so we can
    //     attempt a clean end to the frame
    synchronised = BS_MANCHESTER_ERROR;
  }

  // Now deal with some special conditions in the decoded byte stream

  // First byte is Evo Message header which tells us where payload begins
  if( decoded_bytes==1 && payload_offset==0xFFFF ) {
    if( !decode_error ) {
      // Just received the (header), work out when to expect (len)
      //              hdr  params                           cmd len
      payload_offset = 1 + transcoder_param_len( decoded ) + 2 + 1 ;
    } else {
      synchronised = BS_ABORT; // can't work out packet length now
    }
  }

  // Just decoded the payload len so we can work out how long the radio packet should be
  if( decoded_bytes==payload_offset && rx_pktLen==0xFFFF ) {
    if( !decode_error ) {
      // how many more bytes do we expect
      rx_pktLen  = decoded;   // payload bytes
      rx_pktLen += 1;		  // checksum
      rx_pktLen *= 2;		  // Manchester encoded
      rx_pktLen += 1;		  // EVO_EOF

      // Convert to bits outstanding
      rx_pktLen *= 10; 	      // Convert to bits including start/stop
      rx_pktLen -= rxBits;    // bits still in shift register (accounted for in rx_octets)
      if( rxBits > 7 )
        rx_pktLen += 10;      // next octet has leading stop/start bits
      rx_pktLen += 7;         // allow for padding with bit training

      // And finanly work out the total packet len
      rx_pktLen /= 8;         // convert to octets
      rx_pktLen += rx_octets; // Add in what we've already received

      // Special case return
      return rx_pktLen;	  // Always at least 20: evo_synch[5] hdr[2.5] cmd[5] len[2.5] pl[1.25] csum[2.5] evo_eof[1.25]
    } else {
      synchronised = BS_ABORT;  // Can't work out packet length now
    }
  }

  // Have we reached the end of the packet?
  if( rx_octets==rx_pktLen ) {
    if( rx_checksum != 0 )
      transcoder_rx_status(TC_RX_CHECKSUM_ERROR);

    return BS_END_OF_PACKET;
  }

  if( synchronised==BS_ABORT )
    transcoder_rx_status(TC_RX_ABORTED);

  return synchronised;
}

void bs_rx_rssi( uint8_t rssi)
{
  transcoder_accept_inbound_byte(rssi, TC_RX_RSSI);
}

/****************************************************************************************
* TX byte processing
*
* The CC1101 automatically transmits bit preamble octets and the SYNC1/0 value
*
* At the beginning of a packet we will have to insert the 32 bits of EVO_SYNCH
* Hence the first data byte of the Evo Message should be type 8 and subsequent
* bytes should follow the Even alignement cycle.
*
* The data bytes supplied by transcoder are the little-endian values
* of the Evo Message.  Each byte must be converted to the appropriate 16 bit
* big-endian manchester encoded equivalent and then added to the TX bitsream
* inserting start/stop bits as necessary
*
* If the message does not convert to an exact number of octets we need to 
* append training bits.
*
* When the CC1101 terminates TX it will transmit a [training bit stream]
*
*/

static uint16_t txMsglen; // Length of message to be transmitted
static uint16_t txBytes;  // Number of bytes processed
static uint16_t txPktlen; // Length of packet to be transmitted
static uint16_t txOctets; // Nnmber of octets transferred to FIFO

static rb_t tx_msg;

// Start/stop insertion control
static union shift_register tx;
static uint8_t txBits;   // Number of valid bits in shift register

static inline void insert_p(void)  { tx.data <<= 1 ; tx.data |= 0x01; }
static inline void insert_s(void)  { tx.data <<= 1 ; }
static inline void insert_ps(void) { insert_p(); insert_s(); }
static inline void send( uint8_t n ) { tx.reg <<= n ; }
static uint8_t tx_data(void) {
   txOctets++;
   return cc_put_octet( tx.data );
}

static uint8_t tx_byte( uint8_t byte ) { // convert byte to octets
  uint8_t count = 0;

  tx.bits = byte;

  // For each 4 bytes of data we send 5 octets of bitstream
  // so in each state cycle there is one case which generates
  // two octets
  switch( txBits )
  {
  // Even bit alignment
  case 0: insert_ps(); send(6); count += tx_data(); send(2); txBits=2; break;
  case 2: insert_ps(); send(4); count += tx_data(); send(4); txBits=4; break;
  case 4: insert_ps(); send(2); count += tx_data(); send(6);   // Fall through
  case 6: insert_ps();          count += tx_data();          txBits=8; break;
  case 8:              send(8); count += tx_data();          txBits=0; break;

  // Odd bit alignment
  case 1: insert_p();  send(7); count += tx_data(); send(1); txBits=3; break;
  case 3: insert_ps(); send(5); count += tx_data(); send(3); txBits=5; break;
  case 5: insert_ps(); send(3); count += tx_data(); send(5); txBits=7; break;
  case 7: insert_ps(); send(1); count += tx_data(); send(7);   // Fall through
  case 9: insert_s();           count += tx_data();          txBits=1; break;
  }

  return count;
}

static uint8_t encode_byte( uint8_t msgByte ) {
  uint8_t count=0;
  uint8_t byte0, byte1;

  manchester_encode( msgByte, &byte0, &byte1 );
  count += tx_byte( byte0 );
  count += tx_byte( byte1 );

  return count;
}

/***********************************************************
* Radio TX pull interface
*/

enum tx_state {
  TX_IDLE,        // No TX in progress
  TX_WAITING,     // TX requested but not active yet
  TX_ACTIVE,      // TX frame has started
  TX_IN_FIFO,     // All of data in FIFO
  TX_CLOSING,     // Waiting for end of frame
  TX_MAX
} txState = TX_IDLE;

// Called from radio ISR to see if there's a TX pending
uint8_t bs_enable_tx(void) {
  if( txState == TX_WAITING )
    return 1;

  return 0;
}

// Called from radio ISR at beginning of TX frame
uint16_t bs_start_tx(void) {
  uint16_t pktLen = 0;

  if( txState == TX_WAITING ) {
    pktLen = txPktlen;
    txState = TX_ACTIVE;
  }

  return pktLen;
}

// Called from radio ISR when it allows more data
uint8_t bs_process_tx( uint8_t space ) {

  if( txState == TX_IDLE )
    return 0;

  while( space && ( txOctets < sizeof(evo_header) ) ) {
    // Send Evo header directly to radio
    tx.data = evo_header[txOctets];
    space -= tx_data();
  }

  if( txBytes < txMsglen ) {
    // Transfer message to radio
    while( space > 5 ) {
      // Room for a message byte and Evo Trailer/padding
      if( !rb_empty( &tx_msg ) )
      {
        uint8_t msgByte;

        msgByte = rb_get( &tx_msg );
        space -= encode_byte( msgByte );
        txBytes++;
      }
      else
        break;
    }
  }

  if( txBytes == txMsglen ) {
    // Append Evo trailer
    if( txOctets < txPktlen ) {
      // space test above guarantees room
      tx_byte( EVO_EOF );
      while( txOctets < txPktlen )
        tx_byte(0xAA); // Padding
    }
  }

  if( txOctets == txPktlen ) {
    // Everything is now in FIFO
    if( txState == TX_ACTIVE )
      txState = TX_CLOSING;
  }

  return ( txState == TX_CLOSING );
}

// Called from radio ISR at end of TX frame
void bs_end_tx(void) {
  txState = TX_IDLE;
  // TODO: tell transcoder we've finished
}

/***********************************************************
* Transcoder TX interface
*/

uint8_t bs_send_message( uint16_t msgLen ) {
  uint16_t pktLen;

  if( txState != TX_IDLE )
    return 0;

  pktLen  = msgLen;   // Message bytes
  pktLen *= 2;        // Manchester codes
  pktLen *= 10;       // Message bits
  pktLen += 32;       // Evo Header
  pktLen += 10;       // Evo Trailer (inc start/stop)
  pktLen += 7;        // Padding
  pktLen /= 8;        // octets

  txMsglen = msgLen;
  txBytes = 0;
  txPktlen = pktLen;
  txOctets = 0;
  txBits = 0;  // First thing we need to do is insert Stop/Start

  txState = TX_WAITING;

  cc_tx_trigger();

  return 1;
}

uint8_t bs_send_data( uint8_t msgByte ) {
  if( rb_full( &tx_msg ) )
    return 0;

  rb_put( &tx_msg, msgByte );

  if( txState != TX_IDLE )
    cc_tx_trigger();

  return 1;
}

/******************************************************/

void bs_init(void)
{
  uint8_t i;
  uint32_t hdr ;
  for( i=4, hdr=EVO_SYNCH ; i>0 ; i--, hdr>>=8 )
    evo_header[i-1] = hdr & 0xFF;

  rb_reset( &tx_msg );
  manchester_init();
}

