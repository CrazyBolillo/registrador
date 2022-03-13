#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "pwm.h"

#define INDICATOR_LED PD5
#define IS_RUNNING (PORTD & (1 << INDICATOR_LED))
#define DEBOUNCE_MS 20

#define SERVO_MIN 600
#define SERVO_MAX 2500

void servo_boot_effect();

int main() {
    // PD5, PD6 as output. Pull-ups on PD2, PD3
    PORTD = 0x0C;
    DDRD = 0x60;

    // Enable interrupts on low state for INT0 and INT1 (PD2 & PD3).
    EICRA = 0x00;
    EIMSK = 0x03;
    sei();
    
    setup_pwm();
    servo_boot_effect();

    while (1) {
    }

    return 0;
}

void servo_boot_effect() {
    OCR1A = SERVO_MIN;
    start_pwm();
    _delay_ms(500);
    for (; OCR1A < SERVO_MAX; OCR1A++) { _delay_us(500); }
    for (; OCR1A > SERVO_MIN; OCR1A--) { _delay_us(500); }
    stop_pwm();
}


// Start or stop.
ISR(INT0_vect) {
    _delay_ms(DEBOUNCE_MS);
    if ((PIND & (1 << PD2)) == 0) {
        while ((PIND & (1 << PD2)) == 0);
        PORTD ^= (1 << INDICATOR_LED);
        if (IS_RUNNING != 0) {
            start_pwm();
        }
        else {
            stop_pwm();
        }
    }
}

// Enter setting mode for Servo.
ISR(INT1_vect) {}