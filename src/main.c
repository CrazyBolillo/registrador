#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>

#include "adc.h"
#include "pwm.h"
#include "usart.h"

#define INDICATOR_LED PD5
#define IS_RUNNING (PORTD & (1 << INDICATOR_LED))
#define DEBOUNCE_MS 20

#define SERVO_MIN 600
#define SERVO_MAX 2500

uint16_t adc_read;
unsigned char usart_buffer[32];

void servo_boot_effect();
void adc_to_servo();

int main() {
    start_usart(9600);
    stwrite_usart("Booting up --> v1.0\n");

    start_adc();
    set_adc_ch(0);

    // PD5, PD6 as output. Pull-ups on PD2, PD3
    PORTD = 0x0C;
    DDRD = 0x60;
    
    setup_pwm();
    servo_boot_effect();

    // Enable interrupts on low state for INT0 and INT1 (PD2 & PD3).
    EICRA = 0x00;
    EIMSK = 0x03;
    sei();

    start_pwm();
    while (1) {
        read_adc(&adc_read);
        sprintf(&usart_buffer, "%d\n", adc_read);
        stwrite_usart(usart_buffer);
        adc_to_servo(adc_read);
        _delay_ms(200);
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

void adc_to_servo(uint16_t adc) {
    OCR1A = (2000 / 1023) * adc;
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