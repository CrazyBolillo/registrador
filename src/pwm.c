#include "pwm.h"

/**
 * Setup Timer1 for phase correct PWM mode.
 * Using internal 8M Hz internal clock prescaled by a factor of 8.
 * The generated PWM signal has a frequency of 50 Hz.
 * Output on OC1A (PB1)
*/
void setup_pwm() {
    DDRB |= (1 << PB1);
    TCCR1A = 0xA2;
    TCCR1B = 0x10;
    ICR1 = PWM_TOP;
}

void start_pwm() {
    TCCR1B |= 0x02;
}

void stop_pwm() {
    TCCR1B &= 0xF8;
    PORTB &= ~(1 << PB1);
}