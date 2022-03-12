#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define INDICATOR_LED PD5
#define DEBOUNCE_MS 20

int main() {
    // PD5, PD6 as output. Pull-ups on PD2, PD3
    PORTD = 0x0C;
    DDRD = 0x60;

    DDRB = (1 << PORTB5);

    // Enable interrupts on low state for INT0 and INT1 (PD2 & PD3).
    SREG = 0x80;
    EICRA = 0x00;
    EIMSK = 0x03;
    

    while (1) {
        _delay_ms(1000);
        PORTB ^= (1 << PB5);
    }

    return 0;
}

// Stop or start Timer0 which drives step motor
ISR(INT0_vect) {
    _delay_ms(DEBOUNCE_MS);
    if ((PIND & (1 << PD2)) == 0) {
        while ((PIND & (1 << PD2)) == 0);
        PORTD ^= (1 << INDICATOR_LED);
    }
    
}

// Enter setting mode for Servo.
ISR(INT1_vect) {
    _delay_ms(DEBOUNCE_MS);
    if ((PIND & (1 << PD3)) == 0) {
        while ((PIND & (1 << PD3)) == 0);
        PORTD ^= (1 << INDICATOR_LED);
    }
}