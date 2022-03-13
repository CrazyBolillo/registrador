#include "usart.h"

void start_usart(uint32_t baud) {
    UBRR0 = (F_CPU / (16 * baud)) - 1;
    UCSR0B = 0x08;
    UCSR0C = 0x0E;
}

void write_usart(char data) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
} 

void stwrite_usart(char *data) {
    while(*data != '\0') {
        while (!(UCSR0A & (1<<UDRE0)));
        UDR0 = *data;
        data++;
    }
}