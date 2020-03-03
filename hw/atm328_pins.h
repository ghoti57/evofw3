/**********************************************************
** atm328_pins.h
**
** Abstract pin names and definitions for ATMega 328 family
**
*/

#ifndef _CONFIG_H_
#  error "Include config.h instead of this file"
#endif

#ifndef _HW_ARDUINO_H_
#define _HW_ARDUINO_H_

#include <avr/io.h>

// SPI port defs
#define SPI_PORT    PORTB
#define SPI_PIN     PINB
#define SPI_DDR     DDRB
#define SPI_SS      2
#define SPI_MOSI    3
#define SPI_MISO    4
#define SPI_SCLK    5

// GDO0 connection
#if( GDO0==INT0 )
  #define GDO0_INT_MASK   ( 1 << INT0 )
  #define GDO0_INT_VECT   INT0_vect
  #define GDO0_INT_ISCn0  ISC00
  #define GDO0_INT_ISCn1  ISC01
  #define GDO0_DDR        DDRD
  #define GDO0_PORT       PORTD
  #define GDO0_PIN        PIND
  #define GDO0_IN         ( 1 << PORTD2 )
#elif( GDO0==INT1 )
  #define GDO0_INT_MASK   ( 1 << INT1 )
  #define GDO0_INT_VECT   INT1_vect
  #define GDO0_INT_ISCn0  ISC10
  #define GDO0_INT_ISCn1  ISC11
  #define GDO0_DDR        DDRD
  #define GDO0_PORT       PORTD
  #define GDO0_PIN        PIND
  #define GDO0_IN         ( 1 << PORTD3 )
#else
  #error "GDO0 not mapped"
#endif

// GDO2 connection
#if( GDO2==INT1 )
  #define GDO2_INT_MASK   ( 1 << INT1 )
  #define GDO2_INT_VECT   INT1_vect
  #define GDO2_INT_ISCn0  ISC10
  #define GDO2_INT_ISCn1  ISC11
  #define GDO2_DDR        DDRD
  #define GDO2_PORT       PORTD
  #define GDO2_PIN        PIND
  #define GDO2_IN         ( 1 << PORTD3 )
#elif( GDO2==INT0 )
  #define GDO2_INT_MASK   ( 1 << INT0 )
  #define GDO2_INT_VECT   INT0_vect
  #define GDO2_INT_ISCn0  ISC00
  #define GDO2_INT_ISCn1  ISC01
  #define GDO2_DDR        DDRD
  #define GDO2_PORT       PORTD
  #define GDO2_PIN        PIND
  #define GDO2_IN         ( 1 << PORTD2 )
#else
  #error "GDO2 not mapped"
#endif

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
#define DEBUG_PIN4        ( 1<<PORTC3 )
#define DEBUG_PIN5        ( 1<<PORTC4 )
#define DEBUG_PIN6        ( 1<<PORTC5 )

// TTY USART
#define TTY_UDRE_VECT   USART_UDRE_vect
#define TTY_RX_VECT     USART_RX_vect

// LED
#define LED_DDR   DDRB
#define LED_PORT  PORTB
#define LED_PIN   5

#endif
