#include <avr/interrupt.h>
#include <avr/wdt.h>

extern "C" {
  #include "evo.h"
}

void setup() {
  main_init();
}

void loop() {
  main_work();
}
