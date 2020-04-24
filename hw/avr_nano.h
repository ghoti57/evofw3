#ifndef _CONFIG_H_
#  error "Include config.h instead of this file"
#endif

#ifndef _HW_ARDUINO_H_
#define _HW_ARDUINO_H_

#include <avr/io.h>

#define RINGBUF_SIZE    128

// SPI port defs
#define SPI_PORT    PORTB
#define SPI_PIN     PINB
#define SPI_DDR     DDRB
#define SPI_SS      2
#define SPI_MOSI    3
#define SPI_MISO    4
#define SPI_SCLK    5
#define SPI_CLK_RATE 250000

// GDO0 connection
#define GDO0_INT_MASK   ( 1 << INT0 )
#define GDO0_INT_VECT   INT0_vect
#define GDO0_INT_ISCn0  ISC00
#define GDO0_INT_ISCn1  ISC01
#define GDO0_DDR        DDRD
#define GDO0_PORT       PORTD
#define GDO0_PIN        PIND
#define GDO0_IN         ( 1 << PORTD2 )

// GDO2 connection
#define GDO2_INT_MASK   ( 1 << INT1 )
#define GDO2_INT_VECT   INT1_vect
#define GDO2_INT_ISCn0  ISC10
#define GDO2_INT_ISCn1  ISC11
#define GDO2_DDR        DDRD
#define GDO2_PORT       PORTD
#define GDO2_PIN        PIND
#define GDO2_IN         ( 1 << PORTD3 )

// Software interrupt
#define SW_INT_ENBL     ( 1<<PCIE0 )
#define SW_INT_VECT      PCINT0_vect
#define SW_INT_MASK      PCMSK0
#define SW_INT_PORT      PORTB
#define SW_INT_PIN       PINB
#define SW_INT_DDR       DDRB
#define SW_INT_IN        ( 1<<PORTB0 )

// SOme debug pins
#define DEBUG_PORT        PORTC
#define DEBUG_DDR         DDRC
#define DEBUG_PIN1        ( 1<<PORTC0 )
#define DEBUG_PIN2        ( 1<<PORTC1 )
#define DEBUG_PIN3        ( 1<<PORTC2 )
#define DEBUG_PIN4        ( 1<<PORTC4 )
#define DEBUG_PIN5        ( 1<<PORTC5 )

// TTY USART
#define TTY_UDRE_VECT   USART_UDRE_vect
#define TTY_RX_VECT     USART_RX_vect
#define TTY_BAUD_RATE   115200

// LED
#define LED_DDR   DDRB
#define LED_PORT  PORTB
#define LED_PIN   5

#endif
