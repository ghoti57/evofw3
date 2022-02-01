#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>

extern void spi_init(void);
extern void spi_select( uint8_t ss );

extern void spi_deassert( uint8_t ss );
extern void spi_assert( uint8_t ss );
extern uint8_t spi_check_miso(void);

extern uint8_t spi_send( uint8_t data);
extern uint8_t spi_strobe( uint8_t ss, uint8_t b );

#endif
