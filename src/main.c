#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "pwm.h"
#include "usart.h"

#define INDICATOR_LED PD5
#define IS_RUNNING (PORTD & (1 << INDICATOR_LED))
#define DEBOUNCE_MS 20

#define SERVO_MIN 600
#define SERVO_MAX 2500

void servo_boot_effect();

int main() {
    start_usart(9600);
    stwrite_usart("Booting up --> v1.0\n");

    // PD5, PD6 as output. Pull-ups on PD2, PD3
    PORTD = 0x0C;
    DDRD = 0x60;
    
    setup_pwm();
    servo_boot_effect();

    // Enable interrupts on low state for INT0 and INT1 (PD2 & PD3).
    EICRA = 0x00;
    EIMSK = 0x03;
    sei();

    while (1) {
    }

    return 0;
}

void servo_boot_effect() {
    PORTD |= (1 << INDICATOR_LED);
    OCR1A = SERVO_MIN;
    start_pwm();
    _delay_ms(500);
    for (; OCR1A < SERVO_MAX; OCR1A++) { _delay_us(500); }
    for (; OCR1A > SERVO_MIN; OCR1A--) { _delay_us(500); }
    stop_pwm();
    PORTD &= ~(1 << INDICATOR_LED);
}


// Start or stop.
ISR(INT0_vect) {
    _delay_ms(DEBOUNCE_MS);
    if ((PIND & (1 << PD2)) == 0) {
        while ((PIND & (1 << PD2)) == 0);
        PORTD ^= (1 << INDICATOR_LED);
        if (IS_RUNNING != 0) {
            stwrite_usart("Starting sampling\n");
            start_pwm();
        }
        else {
            stwrite_usart("Stopping sampling\n");
            stop_pwm();
        }
    }
}

// Enter setting mode for Servo.
ISR(INT1_vect) {}