/*********************************************************
*
* cc1101_const.h
* ==============
*
* TI CC1101 radio chip registers
*
*/
#ifndef _CC1101_CONST_H_
#define _CC1101_CONST_H_

#define CC_READ  0x80
#define CC_BURST 0x40

// Read/Write single/burst
#define CC1100_IOCFG2    0x00
#define CC1100_IOCFG1    0x01
#define CC1100_IOCFG0    0x02
#define CC1100_FIFOTHR   0x03
#define CC1100_SYNC1     0x04
#define CC1100_SYNC0     0x05
#define CC1100_PKTLEN    0x06
#define CC1100_PKTCTRL1  0x07
#define CC1100_PKTCTRL0  0x08
#define CC1100_ADDR      0x09
#define CC1100_CHANNR    0x0A
#define CC1100_FSCTRL1   0x0B
#define CC1100_FSCTRL0   0x0C
#define CC1100_FREQ2     0x0D
#define CC1100_FREQ1     0x0E
#define CC1100_FREQ0     0x0F
#define CC1100_MDMCFG4   0x10
#define CC1100_MDMCFG3   0x11
#define CC1100_MDMCFG2   0x12
#define CC1100_MDMCFG1   0x13
#define CC1100_MDMCFG0   0x14
#define CC1100_DEVIATN   0x15
#define CC1100_MCSM2     0x16
#define CC1100_MCSM1     0x17
#define CC1100_MCSM0     0x18
#define CC1100_FOCCFG    0x19
#define CC1100_BSCFG     0x1A
#define CC1100_AGCCTRL2  0x1B
#define CC1100_AGCCTRL1  0x1C
#define CC1100_AGCCTRL0  0x1D
#define CC1100_WOREVT1   0x1E
#define CC1100_WOREVT0   0x1F
#define CC1100_WORCTRL   0x20
#define CC1100_FREND1    0x21
#define CC1100_FREND0    0x22
#define CC1100_FSCAL3    0x23
#define CC1100_FSCAL2    0x24
#define CC1100_FSCAL1    0x25
#define CC1100_FSCAL0    0x26
#define CC1100_RCCTRL1   0x27
#define CC1100_RCCTRL0   0x28
#define CC1100_FSTEST    0x29
#define CC1100_PTEST     0x2A
#define CC1100_AGCTEST   0x2B
#define CC1100_TEST2     0x2C
#define CC1100_TEST1     0x2D
#define CC1100_TEST0     0x2E
#define CC1100_PARAM_MAX 0x2F

// Strobe commands and registers
#define CC1100_SRES      0x30
#define CC1100_SFSTXON   0x31
#define CC1100_SXOFF     0x32
#define CC1100_SCAL      0x33
#define CC1100_SRX       0x34
#define CC1100_STX       0x35
#define CC1100_SIDLE     0x36
#define CC1100_SWORTIME  0x37
#define CC1100_SWOR      0x38
#define CC1100_SPWD      0x39
#define CC1100_SFRX      0x3A
#define CC1100_SFTX      0x3B
#define CC1100_SWORRST   0x3C
#define CC1100_SNOP      0x3D
#define CC1100_PATABLE   0x3E
#define CC1100_FIFO      0x3F

#define CC1100_PA_MAX      8

// Burst mode registers
#define CC1100_PARTNUM        ( CC1100_SRES     | CC_BURST )
#define CC1100_VERSION        ( CC1100_SFSTXON  | CC_BURST )
#define CC1100_FREQEST        ( CC1100_SXOFF    | CC_BURST )
#define CC1100_LQI            ( CC1100_SCAL     | CC_BURST )
#define CC1100_RSSI           ( CC1100_SRX      | CC_BURST )
#define CC1100_MARCSTATE      ( CC1100_STX      | CC_BURST )
#define CC1100_WORTIME1       ( CC1100_SIDLE    | CC_BURST )
#define CC1100_WORTIME0       ( CC1100_SWORTIME | CC_BURST )
#define CC1100_PKTSTATUS      ( CC1100_SWOR     | CC_BURST )
#define CC1100_VCO_VC_DAC     ( CC1100_SPWD     | CC_BURST )
#define CC1100_TXBYTES        ( CC1100_SFRX     | CC_BURST )
#define CC1100_RXBYTES        ( CC1100_SFTX     | CC_BURST )
#define CC1100_RCCTRL1_STATUS ( CC1100_SWORRST  | CC_BURST )
#define CC1100_RCCTRL0_STATUS ( CC1100_SNOP     | CC_BURST )
#define CC1100_RXFIFO         ( CC1100_FIFO     | CC_BURST )
#define CC1100_TXFIFO         ( CC1100_FIFO     | CC_BURST )

// Chip Status byte
#define CC_CHIP_RDY    0x80
#define CC_STATE_MASK  0x70
#define CC_FIFO_MASK   0x0F

#define CC_STATE( status ) ( status & CC_STATE_MASK )
#define CC_STATE_IDLE         0x00
#define CC_STATE_RX           0x10
#define CC_STATE_TX           0x20
#define CC_STATE_FSTXON       0x30
#define CC_STATE_CALIBRATE    0x40
#define CC_STATE_SETTLING     0x50
#define CC_STATE_RX_OVERFLOW  0x60
#define CC_STATE_TX_UNDERFLOW 0x70

#endif // _CC1101_CONST_H_
