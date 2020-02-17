#ifndef _CONFIG_H_
#  error "Include config.h instead of this file"
#endif

#ifndef _HW_SCC_V2_H_
#define _HW_SCC_V2_H_

#define UART_BAUD_RATE   12  // 76800 @ 8MHz
#define RINGBUF_SIZE    128

// SPI port defs
#define SPI_PORT    PORTB
#define SPI_DDR     DDRB
#define SPI_SS      4
#define SPI_MOSI    5
#define SPI_MISO    6
#define SPI_SCLK    7

// Connection to CC1101 GDO2
#define GDO2_CLK_PIN         2
#define GDO2_CLK_INT         INT2
#define GDO2_CLK_INTVECT     INT2_vect
#define GDO2_CLK_INT_ISCn0   ISC20
#define GDO2_CLK_INT_ISCn1   ISC21

// Connection to CC1101 GDO0
#define GDO0_DATA_DDR     DDRB
#define GDO0_DATA_PORT    PORTB
#define GDO0_DATA_PIN     1
#define GDO0_DATA_IN      PINB

// TTY USART
#define TTY_UDRE_VECT   USART0_UDRE_vect
#define TTY_RX_VECT     USART0_RX_vect

// LED
#define LED_DDR   DDRC
#define LED_PORT  PORTC
#define LED_PIN   6

#define NEEDS_MAIN
#define HAS_LED

#endif
