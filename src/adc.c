#include "adc.h"

void start_adc() {
    ADMUX |= (1 << REFS0);
    ADCSRA = 0x87;
}

void set_adc_ch(uint8_t channel) {
    ADMUX &= (0xF0);
    ADMUX |= (channel & 0x0F);
}

void read_adc(uint16_t *read) {
    ADCSRA |= (1 << ADSC);
    while ((ADCSRA & (1 << ADSC)) == 1);
    *read = ADC;
}

void stop_adc() {
   ADCSRA &= ~(1 << ADEN);
}