#ifndef __USART_GUARD__
#define __USART_GUARD__

#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>

void start_usart(uint32_t baud);
void write_usart(char data);
void stwrite_usart(char *data);

#endif