#ifndef _TTY_H_
#define _TTY_H_

#include <stdint.h>

typedef void (*outbound_byte_fn)(uint8_t b);
void tty_init(outbound_byte_fn);
void tty_work(void);

void tty_write_str(char const *s);
void tty_write_char(char c);
void tty_write_hex(uint8_t byte);
 
#endif
