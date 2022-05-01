/* delay.c
 * Written by Connor McKee and Michael Georgariou
 * CPE 316 - Spring 2020
 *
 * For use with the MSP432. */

#include <ti/devices/msp432p4xx/inc/msp.h>
#include "delay.h"

/* set_DCO
 * sets the DCO to run at the inputted frequency */
void set_DCO(uint32_t MHz)
{
    CS -> KEY = CS_KEY_VAL; /* unlock CS */
    CS -> CTL0 = 0;         /* clear out CTL0 */
    /* set CTL1 */
    CS -> CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;

    /* set CTL0 based on passed in frequency */
    if (MHz == FREQ_1_5_MHZ)
        CS -> CTL0 = CS_CTL0_DCORSEL_0;
    else if (MHz == FREQ_3_MHZ)
        CS -> CTL0 = CS_CTL0_DCORSEL_1;
    else if (MHz == FREQ_6_MHZ)
        CS -> CTL0 = CS_CTL0_DCORSEL_2;
    else if (MHz == FREQ_12_MHZ)
        CS -> CTL0 = CS_CTL0_DCORSEL_3;
    else if (MHz == FREQ_24_MHZ)
        CS -> CTL0 = CS_CTL0_DCORSEL_4;
    else if (MHz == FREQ_48_MHZ) {
        /* special case for 48 MHz, from lab manual */
        /* Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */
        while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
            PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
        while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));

        /* Configure Flash wait-state to 1 for both banks 0 & 1 */
        FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL &
            ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) | FLCTL_BANK0_RDCTL_WAIT_1;

        FLCTL->BANK1_RDCTL = (FLCTL->BANK0_RDCTL &
            ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) | FLCTL_BANK1_RDCTL_WAIT_1;

        CS -> CTL0 = CS_CTL0_DCORSEL_5;
        CS -> CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK) |
            CS_CTL1_SELM_3;
    }

    /* lock CS */
    CS -> KEY = 0;
}

/* delay_us
 * Delays the MSP432 the inputted amount of microseconds */
void delay_us(uint32_t us)
{
    int i;

    uint32_t multiplier;

//    if (CS->CTL0 == CS_CTL0_DCORSEL_0) {
//        /* freq = 1.5 MHz */
//        multiplier = 1;
//    }
//    if (CS->CTL0 == CS_CTL0_DCORSEL_1) {
//        /* freq = 3 MHz */
//        multiplier = 3;
//    }
//    if (CS->CTL0 == CS_CTL0_DCORSEL_2) {
//        /* freq = 6 MHz */
//        multiplier = 6;
//    }
//    if (CS->CTL0 == CS_CTL0_DCORSEL_3) {
//        /* freq = 12 MHz */
//        multiplier = 12;
//    }
//    if (CS->CTL0 == CS_CTL0_DCORSEL_4) {
//        /* freq = 24 MHz */
//        multiplier = 24;
//    }
//    if (CS->CTL0 == CS_CTL0_DCORSEL_5) {
//        /* freq = 48 MHz */
//        multiplier = 44;
//    }

    multiplier = 44;

    if (us > 1000) {
        multiplier = multiplier + 1;
    }

    for (i = 0; i < (us/10)*multiplier; i++) {
        ;
    }
}
