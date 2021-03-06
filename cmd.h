/********************************************************
** cmd.h
**
** Debug command processing
**
********************************************************/
#ifndef _CMD_H_
#define _CMD_H_


#define CMD '!'

extern uint8_t cmd( uint8_t byte, char **buffer, uint8_t *n );
extern uint8_t cmd_str( char *str, char **buffer, uint8_t *n );

#endif // _CMD_H_


