#ifndef __PWM_GUARD__
#define __PWM_GUARD__

#include <avr/io.h>

#define PWM_TOP 20000

void setup_pwm();
void start_pwm();
void stop_pwm();

#endif