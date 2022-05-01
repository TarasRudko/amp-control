/* delay.h
 * Written by Connor McKee and Michael Georgariou
 * For use with delay.c for the MSP432
 * CPE 316 - Spring 2020 */

#ifndef DELAY_H
#define DELAY_H
#include <ti/devices/msp432p4xx/inc/msp.h>

#define FREQ_1_5_MHZ 1
#define FREQ_3_MHZ 3
#define FREQ_6_MHZ 6
#define FREQ_12_MHZ 12
#define FREQ_24_MHZ 24
#define FREQ_48_MHZ 48

void set_DCO(uint32_t MHz);
void delay_us(uint32_t us);
void delay_ms(uint32_t delay);
#endif
