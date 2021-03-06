#include <avr/pgmspace.h>
#include <string.h>

#include "nv.h"

#include "cc1101_param.h"

static inline uint8_t cc1100_cfg_default( uint8_t *cfg ) {
  // CC1101 default register settings
  static const uint8_t CC_DEFAULT_CFG[CC1100_PARAM_MAX] PROGMEM = {
    0x0D,  //  CC1100_IOCFG2 	  GDO2- RX data
    0x2E,  //  CC1100_IOCFG1 	  GDO1- not used
    0x2E,  //  CC1100_IOCFG0	  GDO0- TX data
    0x07,  //  CC1100_FIFOTHR   default
    0xD3,  //  CC1100_SYNC1     default
    0x91,  //  CC1100_SYNC0     default
    0xFF,  //  CC1100_PKTLEN	  default
    0x04,  //  CC1100_PKTCTRL1  default
    0x31,  //  CC1100_PKTCTRL0  Asynchornous Serial, TX on GDO0, RX on GDOx
    0x00,  //  CC1100_ADDR      default
    0x00,  //  CC1100_CHANNR    default
    0x0f,  //  CC1100_FSCTRL1   default
    0x00,  //  CC1100_FSCTRL0   default
    0x21,  //  CC1100_FREQ2     /
    0x65,  //  CC1100_FREQ1     / 868.3 MHz
    0x6A,  //  CC1100_FREQ0     /
    0x6A,  //  CC1100_MDMCFG4   //
    0x83,  //  CC1100_MDMCFG3   // DRATE_M=131 data rate=38,383.4838867Hz
    0x10,  //  CC1100_MDMCFG2   GFSK, No Sync Word
    0x22,  //  CC1100_MDMCFG1   FEC_EN=0, NUM_PREAMBLE=4, CHANSPC_E=2
    0xF8,  //  CC1100_MDMCFG0   Channel spacing 199.951 KHz
    0x50,  //  CC1100_DEVIATN   
    0x07,  //  CC1100_MCSM2     default
    0x30,  //  CC1100_MCSM1     default
    0x18,  //  CC1100_MCSM0     Auto-calibrate on Idle to RX+TX, Power on timeout 149-155 uS
    0x16,  //  CC1100_FOCCFG    default
    0x6c,  //  CC1100_BSCFG     default
    0x43,  //  CC1100_AGCCTRL2  
    0x40,  //  CC1100_AGCCTRL1  default
    0x91,  //  CC1100_AGCCTRL0  default
    0x87,  //  CC1100_WOREVT1   default
    0x6B,  //  CC1100_WOREVT0   default
    0xF8,  //  CC1100_WORCTRL   default
    0x56,  //  CC1100_FREND1    default
    0x10,  //  CC1100_FREND0    default
    0xE9,  //  CC1100_FSCAL3
    0x21,  //  CC1100_FSCAL2
    0x00,  //  CC1100_FSCAL1
    0x1f,  //  CC1100_FSCAL0
    0x41,  //  CC1100_RCCTRL1   default
    0x00,  //  CC1100_RCCTRL0   default
    0x59,  //  CC1100_FSTEST    default
    0x7F,  //  CC1100_PTEST     default
    0x3F,  //  CC1100_AGCTEST   default
    0x81,  //  CC1100_TEST2
    0x35,  //  CC1100_TEST1
    0x09,  //  CC1100_TEST0
  };
  uint8_t len=0;
  
  if( cfg ) {
    len = sizeof(CC_DEFAULT_CFG);
    memcpy_P( cfg, CC_DEFAULT_CFG, len );
  }

  return len;
}

static inline uint8_t cc1100_pa_default( uint8_t *paTable ) {
  // CC1101 default Power Ramp settings
  static const uint8_t CC_DEFAULT_PA[CC1100_PA_MAX] PROGMEM = {
    0xC3,0,0,0,0,0,0,0
  };
  uint8_t len=0;

  if( paTable ) {
    len = sizeof(CC_DEFAULT_PA);
    memcpy_P( paTable, CC_DEFAULT_PA, len );
  }

  return len;
}

uint8_t cc_cfg_default( uint8_t param, uint8_t nParam ) {
  uint8_t cfg[CC1100_PARAM_MAX];
  uint8_t len = cc1100_cfg_default( cfg );

  if( param+nParam < len )
    len = nv_param_write( NV_CC_PARAM, param, cfg+param, nParam );

  return len;
}

uint8_t cc_cfg_get( uint8_t param, uint8_t *buff, uint8_t nParam ) {
  uint8_t len=0;

  if( param<CC1100_PARAM_MAX ) {
    len = nv_param_read( NV_CC_PARAM, param, buff, nParam );
    if( len==0 ) {
      uint8_t cfg[CC1100_PARAM_MAX];
      len = cc1100_cfg_default( cfg );
      len = nv_param_write( NV_CC_PARAM, 0, cfg, sizeof(cfg) );
      len = nv_param_read( NV_CC_PARAM, param, buff, nParam );
    }
  }

  return len;
}

uint8_t cc_cfg_set( uint8_t param, uint8_t *buff, uint8_t nParam ) {
  uint8_t len=0;

  if( param+nParam<=CC1100_PARAM_MAX ) {
    len = nv_param_write( NV_CC_PARAM, param, buff, nParam );
  }

  return len;
}

uint8_t cc_pa_default( void ) {
  uint8_t cfg[CC1100_PA_MAX];
  uint8_t len = cc1100_pa_default( cfg );

  return nv_param_write( NV_CC_PA, 0, cfg, len );
}

uint8_t cc_pa_get( uint8_t *paTable ) {
  uint8_t len=0;

  len = nv_param_read( NV_CC_PA, 0, paTable, CC1100_PA_MAX );
  if( len==0 ) {
    len = cc1100_pa_default( paTable );
    len = nv_param_write( NV_CC_PA, 0, paTable, CC1100_PA_MAX );
  }

  for( len=0 ; len<CC1100_PA_MAX ; len++ ) {
    if( paTable[len]==0x00 || paTable[len]==0xFF )
	  break;
  }

  return len;
}
