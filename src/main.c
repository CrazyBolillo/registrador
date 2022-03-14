/**
 * The following program is designed to constantly move a paper roll
 * with the help of a step motor, while simultaneously writing on it with
 * a pen controlled by a servo motor.
 * 
 * -- BOARD --
 * The board used to design the program was an Arduino Nano (ATmega328p), which
 * is why its pin nomenclature is mentioned, even when the program is pure AVR.
 * 
 * -- STEP MOTOR --
 * The program was designed to work with a 28BYJ-48 step motor, controlled 
 * with a ULN2003APG driver. On the driver side, pins IN1:IN4 should be connected
 * to pins D7:D4 (PD7, PD6, PD5 and PD4).
 * The motor speed is controlled by the frequency of Timer0,
 * which can be changed by a varying analog voltage on pin A0 (PC0).
 * 
 * -- SERVO MOTOR --
 * A SG90 servo motor was used for the task, and is what the program is meant
 * to control. It runs at 50Hz and uses pulse widths from 500us to 2500us. 
 * Timer1 generates the PWM signal used to drive it. The analog input from pin
 * A1 (PC1) dictates the servo's angle.
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>

#include "adc.h"
#include "pwm.h"
#include "usart.h"

#define INDICATOR_LED PB0
#define IS_RUNNING (PORTB & (1 << INDICATOR_LED))
#define INDICATOR_ON (PORTB |= (1 << INDICATOR_LED))
#define INDICATOR_OFF (PORTB &= ~(1 << INDICATOR_LED))
#define INDICATOR_TGL PORTB ^= (1 << INDICATOR_LED)
#define DEBOUNCE_MS 20
#define STEPS_LEN 8
#define MIN_STEP_SPEED 55

#define SERVO_MIN 500
#define SERVO_MAX 2500

#define CAL_NONE 'N'
#define CAL_ZERO 'Z'
#define CAL_MAX 'M'

#define CALIB_CH 0
#define READ_CH 1

uint16_t adc_read;
uint16_t usr_servo_min = SERVO_MIN;
uint16_t usr_servo_max = SERVO_MAX;
uint16_t usr_servo_range;
float usr_servo_k;
volatile unsigned char calibrating = 0;
volatile unsigned char cal_status = CAL_NONE;
char usart_buffer[32];
uint8_t curr_step = 0;
unsigned char steps[] = {0x80, 0xC0, 0x40, 0x60, 0x20, 0x30, 0x10, 0x90};

#define PRINT(fmt, args...) sprintf(&usart_buffer, fmt, args); \
stwrite_usart(&usart_buffer);

void setup_usr_servo();
void servo_boot_effect();
uint16_t adc_to_servo();
uint16_t usr_adc_to_servo();
void start();
void stop();

int main() {
    start_usart(9600);
    stwrite_usart("Booting up --> v1.0\n");

    /**
     * -- GPIO Setup -- 
     * PD2 and PD3 are used for external interrupts (INT0, INT1) so they are
     * set as inputs with internal pull-ups enabled.
     * 
     * Pins D8:D4 are outputs used to control the step motor and indicator
     * LED so they are set as outputs.
     * PB0 is D9 and PD7:PD4 are pins D7:D4 on Arduino.
    */
    PORTB &= 0xFE;
    DDRB |= 0x01;
    PORTD = 0x0C;
    DDRD |= 0xF0;
    
    setup_pwm();
    servo_boot_effect();
    setup_usr_servo();

    // Enable interrupts on low state for INT0 and INT1 (PD2 & PD3).
    EICRA = 0x00;
    EIMSK = 0x03;

    OCR0A = 128;
    TCCR0A = 0x02;
    TCCR0B = 0x00;
    TIMSK0 = 0x02;

    sei();

    while (1) {
        if (calibrating == 1) {
            stwrite_usart("Starting calibration...\n");
            stwrite_usart("Set minimum angle\n");

            start_adc();
            set_adc_ch(CALIB_CH);
            OCR1A = SERVO_MIN;
            start_pwm();
            while(cal_status == CAL_ZERO) {
                read_adc(&adc_read);
                usr_servo_min = adc_to_servo();
                OCR1A = usr_servo_min;
            }
            stwrite_usart("Set max angle\n");
            while (cal_status == CAL_MAX) {
                read_adc(&adc_read);
                usr_servo_max = adc_to_servo();
                OCR1A = usr_servo_max;
            }
            calibrating = 0;
            cal_status = CAL_NONE;
            PRINT("--- Finished calibration ---\nMin: %d -- Max: %d\n", 
                usr_servo_min, usr_servo_max
            );
            setup_usr_servo();
            OCR1A = usr_servo_min;
            _delay_ms(500);
            OCR1A = usr_servo_max;
            _delay_ms(500);
            OCR1A = usr_servo_min;
            _delay_ms(500);
            stop_pwm();
        }
        else if (IS_RUNNING != 0) {
            set_adc_ch(CALIB_CH);
            read_adc(&adc_read);
            uint16_t speed = (adc_read / 4) - 1;
            if (speed >= MIN_STEP_SPEED) {
                OCR0A = speed;
            }
            else {
                OCR0A = MIN_STEP_SPEED;
            }
            set_adc_ch(READ_CH);
            read_adc(&adc_read);
            OCR1A = usr_adc_to_servo();
        }
    }

    return 0;
}

/**
 * Perform the necessary calculations to correctly move the servo motor later on, based on user
 * configuration.
*/
void setup_usr_servo() { 
    usr_servo_range = usr_servo_max - usr_servo_min;
    usr_servo_k = usr_servo_range / 1023;
}

/**
 * Boot up effect used on startup to verify servo's functioning.
*/
void servo_boot_effect() {
    INDICATOR_ON;
    OCR1A = SERVO_MIN;
    start_pwm();
    for (; OCR1A < SERVO_MAX; OCR1A++) { _delay_us(150); }
    for (; OCR1A > SERVO_MIN; OCR1A--) { _delay_us(150); }
    _delay_ms(500);
    for (; OCR1A < SERVO_MAX; OCR1A++) { _delay_us(150); }
    for (; OCR1A > SERVO_MIN; OCR1A--) { _delay_us(150); }
    stop_pwm();
    INDICATOR_OFF;
}

/**
 * Convert an ADC reading to the pulse width required to obtain a certain
 * angle in a servo motor. 
 * An ADC reading of 0 is equal to a 0 degree angle.
 * An ADC reading of 1023 is equal to a 180 degrees angle.
*/
uint16_t adc_to_servo() {
    return (2 * adc_read) + SERVO_MIN;
}

/**
 * Convert an ADC reading to the pulse width required to obtain a certain angle in a servo motor. Takes in account
 * user defined max and min angles.
 * An ADC reading of 0 is equal to the minimum angle set by the user.
 * An ADC reading of 1023 is equal to the max angle set by the user.
*/
uint16_t usr_adc_to_servo() {
    return (usr_servo_k * adc_read) + usr_servo_min;
}

void start() {
    stwrite_usart("Starting sampling\n");
    INDICATOR_ON;
    start_adc();
    TIMSK0 = 0x02;
    TCCR0B |= 0x04;
    start_pwm();
}

void stop() {
    TIMSK0 = 0x00;
    stwrite_usart("Stopping sampling\n");
    INDICATOR_OFF;
    stop_adc();
    TCCR0B &= 0xF8;
    stop_pwm();
}

/**
 * Interrupt vector to start or stop sampling. Activated when INT0 
 * (D2 on Arduino) is sent into a low state.
*/
ISR(INT0_vect) {
    _delay_ms(DEBOUNCE_MS);
    if ((PIND & (1 << PD2)) == 0) {
        while ((PIND & (1 << PD2)) == 0);
        INDICATOR_TGL;
        if (IS_RUNNING != 0) {
            start();
        }
        else {
            stop();
        }
    }
}

/**
 * Interrupt vector to stop sampling and enter into calibration mode, where
 * the user can set the angle the servo should be in for a zero reading, and
 * the angle for a max reading.
 * Activated when INT1 (D3 on Arduino) is sent into a low state.
*/
ISR(INT1_vect) {
    _delay_ms(DEBOUNCE_MS);
    if ((PIND & (1 << PD3)) == 0) {
        while((PIND & (1 << PD3)) == 0);
        INDICATOR_OFF;
        calibrating = 1;
        if (cal_status == CAL_NONE) {
            cal_status = CAL_ZERO;
        }
        else if (cal_status == CAL_ZERO) {
            cal_status = CAL_MAX;
        }
        else {
            cal_status = CAL_ZERO;
        }
    }
}

/**
 * Interrupt vector to turn the step motor. With every interrupt
 * the motor is advanced one step.
*/
ISR(TIMER0_COMPA_vect) {
    if (curr_step == STEPS_LEN) {
        curr_step = 0;
    }
    PORTD = ((steps[curr_step]) | (PORTD & 0x0F));
    curr_step++;
}