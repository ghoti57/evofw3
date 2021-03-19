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
#define SPI_SS      CCSEL
#define SPI_MOSI    2
#define SPI_MISO    3
#define SPI_SCLK    1

// GDO0 connection
#if( GDO0==INT2 )
  #define GDO0_DDR        DDRD
  #define GDO0_PORT       PORTD
  #define GDO0_PIN        PIND
  #define GDO0_IN         ( 1 << PD2 )
#elif( GDO0==INT3 )
  #define GDO0_DDR        DDRD
  #define GDO0_PORT       PORTD
  #define GDO0_PIN        PIND
  #define GDO0_IN         ( 1 << PD3 )
#else
  #error "GDO0 not mapped"
#endif

// GDO2 connection
#if( GDO2==INT3 )
  #define GDO2_DDR        DDRD
  #define GDO2_PORT       PORTD
  #define GDO2_PIN        PIND
  #define GDO2_IN         ( 1 << PD3 )
#elif( GDO2==INT2 )
  #define GDO2_DDR        DDRD
  #define GDO2_PORT       PORTD
  #define GDO2_PIN        PIND
  #define GDO2_IN         ( 1 << PD2 )
#else
  #error "GDO2 not mapped"
#endif

// GDO0 interrupt
#if( GDO0==INT2 )
  #define GDO0_INT_MASK   ( 1 << INT2 )
  #define GDO0_INT_VECT   INT2_vect
  #define GDO0_INT_ISCn0  ( 1 << ISC20 )
  #define GDO0_INT_ISCn1  ( 1 << ISC21 )
#elif( GDO0==INT3 )
  #define GDO0_INT_MASK   ( 1 << INT3 )
  #define GDO0_INT_VECT   INT3_vect
  #define GDO0_INT_ISCn0  ( 1 << ISC30 )
  #define GDO0_INT_ISCn1  ( 1 << ISC31 )
#else
  #error "GDO2 not mapped"
#endif

// GDO2 interrupt
#if( GDO2==INT3 )
  #define GDO2_INT_MASK   ( 1 << INT3 )
  #define GDO2_INT_VECT   INT3_vect
  #define GDO2_INT_ISCn0  ( 1 << ISC30 )
  #define GDO2_INT_ISCn1  ( 1 << ISC31 )
#elif( GDO2==INT2 )
  #define GDO2_INT_MASK   ( 1 << INT2 )
  #define GDO2_INT_VECT   INT2_vect
  #define GDO2_INT_ISCn0  ( 1 << ISC20 )
  #define GDO2_INT_ISCn1  ( 1 << ISC21 )
#else
  #error "GDO2 not mapped"
#endif

#if defined(SWUART)
// Software interrupt
#define SW_INT_ENBL     ( 1<<PCIE0 )
#define SW_INT_VECT      PCINT0_vect
#define SW_INT_MASK      PCMSK0
#define SW_INT_PORT      PORTB
#define SW_INT_PIN       PINB
#define SW_INT_DDR       DDRB
#define SW_INT_IN        ( 1<<PORTB4 )
#endif // SWUART

// Some debug pins
#define DEBUG_PORT        PORTF
#define DEBUG_DDR         DDRF
#define DEBUG_PIN1        ( 1<<PORTF7 )
#define DEBUG_PIN2        ( 1<<PORTF6 )
#define DEBUG_PIN3        ( 1<<PORTF5 )
#define DEBUG_PIN4        ( 1<<PORTF4 )
#define DEBUG_PIN5        ( 1<<PORTF1 )
#define DEBUG_PIN6        ( 1<<PORTF0 )

// TTY USB
#define TTY_USB

// LED
//#define LED_DDR   DDRB
//#define LED_PORT  PORTB
//#define LED_PIN   5

#endif
