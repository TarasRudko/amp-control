/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//****************************************************************************
//
// main.c - MSP-EXP432P401R + Educational Boosterpack MkII - Temperature
//
//          Displays temperature measured by the TMP006 Infrared Thermopile
//          Contactless Temperature Sensor. The MSP432 communicates
//          with the sensor through I2C.
//
//****************************************************************************


#include <LcdDriver/tft-2_2-240x320_ILI9341.h>
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.h"
#include "HAL_I2C.h"
#include "HAL_TMP006.h"
#include <stdio.h>
#include <trizub-240x320.h>
#include "fonts/fontCourier15x24.h"
#include <ti/devices/msp432p4xx/driverlib/systick.h>

#define DEGREE_CHARACTER 223
#define DEFAULT_VOLUME 100

/* Graphic library context */
Graphics_Context g_sContext;

/* Variable for storing temperature value returned from TMP006 */
float temp;
uint8_t volume=DEFAULT_VOLUME;
uint8_t old_volume = DEFAULT_VOLUME;

Graphics_Rectangle rect;
Graphics_Rectangle rect_yellow, rectFullScreen;

/*
 * Main function
 */
int main(void)
{


    rect.xMax=319;
    rect.xMin=0;
    rect.yMax=119;
    rect.yMin=0;
    rect_yellow.xMax=319;
    rect_yellow.xMin=0;
    rect_yellow.yMax=239;
    rect_yellow.yMin=120;
    rectFullScreen.xMax=319;
    rectFullScreen.xMin=0;
    rectFullScreen.yMax=239;
    rectFullScreen.yMin=0;

    /* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);



    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_RIGHT);


    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);

    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
//    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);

    Crystalfontz128x128_DrawBitmap(&g_sCrystalfontz128x128, &rectFullScreen, &image_data_trizub240x320);



    /* Configuring P1.0 as LED */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Configuring P4.0 as an input and enabling interrupts */
    MAP_GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN0);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN0, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN0);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN0);

    MAP_GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN1);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN1, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN1);
    MAP_Interrupt_enableInterrupt(INT_PORT4);

    // on 48MGz 1 tick = 0.0208333 us
    // set period to 0.1s
    MAP_SysTick_enableModule();
    SysTick_setPeriod(4800000);
    MAP_SysTick_enableInterrupt();

    /* Enabling MASTER interrupts */
    MAP_Interrupt_enableMaster();
    /* Initializes graphics context */

    __delay_cycles(100000);

    Graphics_setFont(&g_sContext, &g_sFontcourier15x24);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);

    char volume_str[20];
    sprintf(volume_str, "Volume = ");

    Graphics_drawStringCentered(&g_sContext,
                                (int8_t *)volume_str,
                                AUTO_STRING_LENGTH,
                                120,
                                30,
                    OPAQUE_TEXT);
    sprintf(volume_str, "%5.1f dB",31.5-(0.5*(255-volume)));
    Graphics_drawStringCentered(&g_sContext,
                                (int8_t *)volume_str,
                                45,
                                220,
                                30,
                    OPAQUE_TEXT);
    while(1)
    {

    }
}

/* SysTick ISR */
void SysTick_Handler(void)
{
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    if (old_volume != volume) {
        char volume_str[20];

        sprintf(volume_str, "%5.1f dB",31.5-(0.5*(255-volume)));

        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)volume_str,
                                    45,
                                    220,
                                    30,
                        OPAQUE_TEXT);
        old_volume = volume;
    }
}

/* GPIO ISR - Encoder */
void PORT4_IRQHandler(void)
{

    // Encoder interrupt routine for both pins. Updates counter
    // if they are valid and have rotated a full indent

    static uint8_t old_AB = 3;  // Lookup table index
    static int8_t encval = 0;   // Encoder value
    static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

    old_AB <<=2;  // Remember previous state
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);

    if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN0)) old_AB |= 0x02; // Add current state of pin A
    if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN1)) old_AB |= 0x01; // Add current state of pin B

    encval += enc_states[( old_AB & 0x0f )];

    // Update counter if encoder has rotated a full indent, that is at least 4 steps
    if( encval > 3 ) {        // Four steps forward
      if (volume>0) volume--;              // Increase counter
      encval = 0;
    }
    else if( encval < -3 ) {  // Four steps backwards
     if (volume<255) volume++;               // Decrease counter
     encval = 0;
    }

}
