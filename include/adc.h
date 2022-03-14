#ifndef __ADC_GUARD__
#define __ADC_GUARD__

#include <avr/io.h>
#include <stdint.h>

void start_adc();
void set_adc_ch(uint8_t channel);
void read_adc(volatile uint16_t *read);
void stop_adc();


#endif