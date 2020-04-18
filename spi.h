#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>

extern void spi_init(void);

extern void spi_deassert(void);
extern void spi_assert(void);
extern uint8_t spi_check_miso(void);

extern uint8_t spi_send(uint8_t data);
extern uint8_t spi_strobe(uint8_t b);

#endif
