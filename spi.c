#include "config.h"

#include "spi.h"

static void spi_set_clock( uint32_t Fosc, uint32_t sckFreq )
{
  // ATMEGA328 data sheet Table23-5 
  // Relationship between SCK and Oscillator Frequency
  static struct spi_sck_options {
    uint8_t spi2x :1;
    uint8_t spr1  :1;
    uint8_t spr0  :1;
  } const sck_options[8] = {
    { 1,0,0 },  // Fosc/2^0 (1) - not supported - defaults to Fosc/2
    { 1,0,0 },  // Fosc/2^1 (2)   max SCK frequency
    { 0,0,0 },  // Fosc/2^2 (4)
    { 1,0,1 },  // Fosc/2^3 (8)
    { 0,0,1 },  // Fosc/2^4 (16)
    { 1,1,0 },  // Fosc/2^5 (32)
    { 0,1,0 },  // Fosc/2^6 (64)
    { 0,1,1 }   // Fosc/2^7 (128) min SCK frequency
  }, *opt;

  uint8_t power=0;
  while( sckFreq < Fosc ) {
    sckFreq <<= 1;
    power++;
  }
  if( power>7 ) power = 7;
  opt = &sck_options[power];

  SPSR &= ~( 1 << SPI2X );
  SPCR &= ~( 1 << SPR1 );
  SPCR &= ~( 1 << SPR0 );

  SPSR |= opt->spi2x << SPI2X ;
  SPCR |= opt->spr1 << SPR1 ;
  SPCR |= opt->spr0 << SPR0 ;
}

void spi_init(void) {
  SPI_PORT |= ( 1 << SPI_SCLK );

  SPI_DDR |= ( 1 << SPI_MOSI ) | ( 1 << SPI_SCLK ) | ( 1 << SPI_SS );
  SPI_DDR &= ~(1 << SPI_MISO);

  SPCR = 0;
  spi_set_clock( F_CPU,  SPI_CLK_RATE );
  SPCR |= ( 1 << MSTR ) | ( 1 << SPE ) ;
}

void spi_deassert(void) {
  SPI_PORT |= (1 << SPI_SS);
}

void spi_assert(void) {
  SPI_PORT &= ~(1 << SPI_SS);
}

uint8_t spi_check_miso(void) {
  return ( SPI_PIN & (1 << SPI_MISO) );
}

uint8_t spi_send(uint8_t data) {
  SPDR = data;
  while (!(SPSR & (1 << SPIF)));
  return SPDR;
}

uint8_t spi_strobe(uint8_t b) {
  uint8_t result;
  spi_assert();
  while( spi_check_miso() );
  result = spi_send(b);
  while( spi_check_miso() );
  spi_deassert();
  return result;
}

