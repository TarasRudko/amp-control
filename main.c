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
#include <stdio.h>
#include <trizub-240x320.h>
#include "fonts/fontCourier15x24.h"
#include <ti/devices/msp432p4xx/driverlib/systick.h>
#include <ti/grlib/button.h>


#define COLOR_DARK_GREY 0x333333
#define INACTIVE_TEXT_COLOR COLOR_DARK_GREY
#define ACTIVE_TEXT_COLOR GRAPHICS_COLOR_LIME_GREEN
#define DEGREE_CHARACTER 223
#define DEFAULT_VOLUME 100
#define LEVEL_BAR_X0 0
#define LEVEL_BAR_X1 320
#define LEVEL_BAR_Y0 195
#define LEVEL_BAR_Y1 239
#define LEVEL_BAR_BG_COLOR GRAPHICS_COLOR_BLACK
#define LEVEL_BAR_LOW_COLOR GRAPHICS_COLOR_LIME_GREEN
#define LEVEL_BAR_HIGH_COLOR GRAPHICS_COLOR_ORANGE_RED
#define LEVEL_BAR_COLOR_INACTIVE  COLOR_DARK_GREY
#define LEVEL_HIGH_TRESHHOLD 10
#define LEVEL_MAX 11
#define LEVEL_HORIZONTAL_OFFSET 30
#define LEVEL_VERTICAL_OFFSET 5
#define LEVEL_ELEMENT_WIDHT 14
#define LEVEL_ELEMENT_HEIGHT 5
#define LEVEL_ELEMENT_SPACE_H 10
#define LEVEL_ELEMENT_SPACE_V 25
#define LEVEL_TEXT_H_OFFSET 2
#define LEVEL_TEXT_DB_V_OFFSET 15
#define INPUTS_TXT_X 5
#define INPUTS_TXT_Y 5
#define INPUTS_TXT_Y_OFFSET 25

// TIMER, IR RC
#define STARTBITMIN 250
#define STARTBITMAX 330
#define PAUSEBITMIN 130
#define PAUSEBITMAX 160
#define BITMIN 10
#define BITMAX 30
#define ZEROBITMIN 10
#define ZEROBITMAX 30
#define ONEBITMIN 50
#define ONEBITMAX 60
#define IRREPEAT 2
#define IRDONE 1
#define IRNONE 0



#define AMP_CTL_delay(x)      __delay_cycles(x * 48)

enum inputs{ input_CD, input_DAC, input_PC };


/* Graphic library context */
Graphics_Context g_sContext;

/* Variable for storing temperature value returned from TMP006 */
float temp;


uint8_t level_left = 0;
uint8_t level_right = 0;
bool refresf_volume=true;
uint8_t volume=DEFAULT_VOLUME;
bool OK_button_redraw=true;
bool input_changed=true;
enum inputs source_input = input_CD;
Graphics_Rectangle rect;
Graphics_Rectangle rect_yellow, rectFullScreen;
Graphics_Button OK_button;
char OK_button_text[4];

// RC
int x=0;
int IRdatatmp, IRdata;
int IRready=0;
int TimeInterval=0;
uint16_t IRaddress;
uint8_t IRcommand;
void DrawSignalLevelInit(void);
void DrawSignalLevel(void);
void initADC();
void initSoundLevelADC();
void getSampleSoundLevel(uint8_t *X, uint8_t *Y);
void portSetup(void);
void DrawInitialScreen(void);
void ReDrawScreen(void);
void InitClock(void);
void DrawInputSelector();
void setupTimer();

/* Timer_A Capture Mode Configuration Parameter */
const Timer_A_CaptureModeConfig captureModeConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,         // CC Register 2
        TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,          // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxB,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

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

    sprintf(OK_button_text, "OK");
    OK_button.xMin = 50;
    OK_button.xMax = 110;
    OK_button.yMin = 50;
    OK_button.yMax = 100;
    OK_button.borderColor = GRAPHICS_COLOR_BLACK;
    OK_button.borderWidth = 2;
    OK_button.fillColor = GRAPHICS_COLOR_GRAY;
    OK_button.font = &g_sFontCmss22b;
    OK_button.selected = false;
    OK_button.selectedColor = GRAPHICS_COLOR_BLACK;
    OK_button.selectedTextColor = GRAPHICS_COLOR_BLUE;
    OK_button.text = (int8_t *)OK_button_text;
    OK_button.textColor = GRAPHICS_COLOR_BLACK;
    OK_button.textXPos = 55;
    OK_button.textYPos = 55;
    /* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

    InitClock();

    initADC();
    initSoundLevelADC();


    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_RIGHT);


    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);

    /* Initializes graphics context */

    __delay_cycles(100000);

    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
//    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);

    Crystalfontz128x128_DrawBitmap(&g_sCrystalfontz128x128, &rectFullScreen, &image_data_trizub240x320);

    portSetup();
    setupTimer();


    // on 48MGz 1 tick = 0.0208333 us
    // set period to 0.1s
    MAP_SysTick_enableModule();
    SysTick_setPeriod(48000000);
    MAP_SysTick_enableInterrupt();

    DrawInitialScreen();

    /* Enabling MASTER interrupts */
    MAP_Interrupt_enableMaster();

    /* Starting the Timer_A0 in continuous mode */
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);

    while(1)
    {

        if (IRready == IRDONE) {
            MAP_Interrupt_disableMaster();
            char command_str[30];
            sprintf(command_str, "IRCommand: %X", IRcommand);
            Graphics_setForegroundColor(&g_sContext, ACTIVE_TEXT_COLOR);

            Graphics_drawString(&g_sContext,
                                        (int8_t *)command_str,
                                        AUTO_STRING_LENGTH,
                                        20,
                                        100,
                            OPAQUE_TEXT);
            IRready = IRNONE;
            MAP_Interrupt_enableMaster();
        }
    }
}

/* SysTick ISR */
void SysTick_Handler(void)
{
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

    getSampleSoundLevel(&level_left, &level_right);

    ReDrawScreen();
}

/* GPIO ISR - Encoder */
void PORT4_IRQHandler(void)
{
    uint32_t status;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);

    if (status & GPIO_PIN2 ) {
        // Encoder has been pushed
        source_input =(source_input + 1) % 3;
        input_changed = true;
        OK_button.selected = true;
        OK_button_redraw = true;
        AMP_CTL_delay(5000);
        return;
    }
    // Encoder interrupt routine for both pins. Updates counter
    // if they are valid and have rotated a full indent

    static uint8_t old_AB = 3;  // Lookup table index
    static int8_t encval = 0;   // Encoder value
    static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

    old_AB <<=2;  // Remember previous state

    if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN0)) old_AB |= 0x02; // Add current state of pin A
    if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN1)) old_AB |= 0x01; // Add current state of pin B

    encval += enc_states[( old_AB & 0x0f )];

    // Update counter if encoder has rotated a full indent, that is at least 4 steps
    if( encval > 3 ) {        // Four steps forward
      if (volume>0) volume--;              // Increase counter
      encval = 0;
      refresf_volume = true;
    }
    else if( encval < -3 ) {  // Four steps backwards
     if (volume<255) volume++;               // Decrease counter
     encval = 0;
     refresf_volume = true;
    }

}

void DrawSignalLevelInit(void){
    Graphics_Rectangle rect;
    rect.xMax=LEVEL_BAR_X1;
    rect.xMin=LEVEL_BAR_X0;
    rect.yMax=LEVEL_BAR_Y1;
    rect.yMin=LEVEL_BAR_Y0;
    Graphics_setForegroundColor(&g_sContext, LEVEL_BAR_BG_COLOR);
    Graphics_fillRectangle(&g_sContext, &rect);
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    char sound_level_str[50];
    Graphics_setForegroundColor(&g_sContext, LEVEL_BAR_LOW_COLOR);
    sprintf(sound_level_str, "dB  -20     -15     -10      -6      -3     0    +2");

    Graphics_drawString(&g_sContext,
                                (int8_t *)sound_level_str,
                                AUTO_STRING_LENGTH,
                                LEVEL_BAR_X0 + LEVEL_TEXT_H_OFFSET,
                                LEVEL_BAR_Y0 + LEVEL_TEXT_DB_V_OFFSET,
                    OPAQUE_TEXT);
}



void DrawSignalLevel(void) {
    Graphics_Rectangle rect;
    uint8_t ii;
    for (ii=0; ii<=LEVEL_MAX; ii++ ){
        if (ii < LEVEL_HIGH_TRESHHOLD)  {
            Graphics_setForegroundColor(&g_sContext, LEVEL_BAR_LOW_COLOR);
        }
        else {
            Graphics_setForegroundColor(&g_sContext, LEVEL_BAR_HIGH_COLOR);
        }
        if ( ii > level_left ) Graphics_setForegroundColor(&g_sContext, LEVEL_BAR_COLOR_INACTIVE);
        rect.xMin=ii*LEVEL_ELEMENT_WIDHT+ii* LEVEL_ELEMENT_SPACE_H + LEVEL_HORIZONTAL_OFFSET;
        rect.xMax=rect.xMin+LEVEL_ELEMENT_WIDHT;

        rect.yMin=LEVEL_BAR_Y0 + LEVEL_VERTICAL_OFFSET;
        rect.yMax=rect.yMin + LEVEL_ELEMENT_HEIGHT;
        Graphics_fillRectangle(&g_sContext, &rect);

        if (ii < LEVEL_HIGH_TRESHHOLD)  {
            Graphics_setForegroundColor(&g_sContext, LEVEL_BAR_LOW_COLOR);
        }
        else {
            Graphics_setForegroundColor(&g_sContext, LEVEL_BAR_HIGH_COLOR);
        }
        if ( ii > level_right ) Graphics_setForegroundColor(&g_sContext, LEVEL_BAR_COLOR_INACTIVE);

        rect.yMin=LEVEL_BAR_Y0 + LEVEL_VERTICAL_OFFSET + LEVEL_ELEMENT_SPACE_V;
        rect.yMax=rect.yMin + LEVEL_ELEMENT_HEIGHT;
        Graphics_fillRectangle(&g_sContext, &rect);
    }
}

void initADC() {
    ADC14_enableModule();

    // This sets the conversion clock to 3MHz
    ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC,
                     ADC_PREDIVIDER_1,
                     ADC_DIVIDER_1,
                     0
                     );

    // This configures the ADC to store output results
    // in ADC_MEM0 up to ADC_MEM1. Each conversion will
    // thus use two channels.
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);

    // This configures the ADC in manual conversion mode
    // Software will start each conversion.
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
}

void initSoundLevelADC() {

    // This configures ADC_MEM0 to store the result from
    // input channel A15 (Joystick X), in non-differential input mode
    // (non-differential means: only a single input pin)
    // The reference for Vref- and Vref+ are VSS and VCC respectively
    ADC14_configureConversionMemory(ADC_MEM0,
                                  ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                  ADC_INPUT_A15,                 // joystick X
                                  ADC_NONDIFFERENTIAL_INPUTS);

    // This selects the GPIO as analog input
    // A15 is multiplexed on GPIO port P6 pin PIN0
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
                                               GPIO_PIN0,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

    // This configures ADC_MEM0 to store the result from
    // input channel A15 (Joystick X), in non-differential input mode
    // (non-differential means: only a single input pin)
    // The reference for Vref- and Vref+ are VSS and VCC respectively
    ADC14_configureConversionMemory(ADC_MEM1,
                                    ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                    ADC_INPUT_A9,                 // joystick Y
                                    ADC_NONDIFFERENTIAL_INPUTS);

    // This selects the GPIO as analog input
    // A9 is multiplexed on GPIO port P4 pin PIN4
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                                               GPIO_PIN4,
                                               GPIO_TERTIARY_MODULE_FUNCTION);
}

void getSampleSoundLevel(uint8_t *X, uint8_t *Y) {
    // This starts the conversion process
    // The S/H will be followed by SAR conversion
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();

    // We wait for the ADC to complete
    while (ADC14_isBusy()) ;

    // and we read the output result from buffer ADC_MEM0
    *X = ADC14_getResult(ADC_MEM0)*12/0x4000;
    *Y = ADC14_getResult(ADC_MEM1)*12/0x4000;

}

void portSetup(void) {
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

        // Encoder switch
        MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN2);
    //    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);

        /* Configuring P2.4 as peripheral input for capture IR signal */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);


        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN2);
        MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN2);
        MAP_Interrupt_enableInterrupt(INT_TA0_N);
        MAP_Interrupt_enableInterrupt(INT_PORT4);
}

void DrawInitialScreen(void){
        DrawSignalLevelInit();
        DrawSignalLevel();
        Graphics_setFont(&g_sContext, &g_sFontcourier15x24);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
        Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
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


}

void ReDrawScreen(void){
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setFont(&g_sContext, &g_sFontcourier15x24);
    char volume_str[20];
    if (refresf_volume) {
        sprintf(volume_str, "%5.1f dB",31.5-(0.5*(255-volume)));
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)volume_str,
                                    45,
                                    220,
                                    30,
                        OPAQUE_TEXT);
        refresf_volume = false;
    }
    DrawInputSelector();
    DrawSignalLevel();
    if (OK_button_redraw) {
        Graphics_drawButton( &g_sContext, &OK_button);
        OK_button_redraw = false;
        OK_button.selected = false;
    }

}

void InitClock(void){
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
}

void DrawInputSelector(){

    Graphics_setFont(&g_sContext, &g_sFontCmss22b);

    char input_str[11];
    uint32_t CD_txt_color, DAC_txt_color, PC_txt_color;
    CD_txt_color = INACTIVE_TEXT_COLOR;
    DAC_txt_color = INACTIVE_TEXT_COLOR;
    PC_txt_color= INACTIVE_TEXT_COLOR;
    if (input_changed){
        switch (source_input) {
            case input_CD:
                CD_txt_color = ACTIVE_TEXT_COLOR;
                sprintf(input_str, "Input: CD ");
              break;

            case input_DAC:
              // statements
                sprintf(input_str, "Input: DAC");
                DAC_txt_color = ACTIVE_TEXT_COLOR;
              break;

            case input_PC:
              // statements
                sprintf(input_str, "Input: PC ");
                PC_txt_color = ACTIVE_TEXT_COLOR;
              break;
        }

        Graphics_setForegroundColor(&g_sContext, CD_txt_color);
        Graphics_drawString(&g_sContext,
                            (int8_t *)"CD",
                            AUTO_STRING_LENGTH,
                            INPUTS_TXT_X,
                            INPUTS_TXT_Y,
                            TRANSPARENT_TEXT);
        Graphics_setForegroundColor(&g_sContext, DAC_txt_color);
        Graphics_drawString(&g_sContext,
                            (int8_t *)"DAC",
                            AUTO_STRING_LENGTH,
                            INPUTS_TXT_X,
                            INPUTS_TXT_Y + INPUTS_TXT_Y_OFFSET,
                            TRANSPARENT_TEXT);
        Graphics_setForegroundColor(&g_sContext, PC_txt_color);
        Graphics_drawString(&g_sContext,
                            (int8_t *)"PC",
                            AUTO_STRING_LENGTH,
                            INPUTS_TXT_X,
                            INPUTS_TXT_Y + 2*INPUTS_TXT_Y_OFFSET,
                            TRANSPARENT_TEXT);
        input_changed = false;
      }

}

void setupTimer(){
    TA0CCTL1 = CM_3 + CCIS_0 + SCS + CAP + CCIE; // CM_3 - Capture both edges
    TA0CCR2 = 500;  // Error detection timeout
    TA0CCTL2 = CM_1 + CCIE;  //CM_1 - Caprute on rising edge, CCIE -capture/compare interrupt enable
    TA0CTL = TASSEL_1 + MC_2 + TAIE + ID_0; // ACLK = 32768Hz
}

void TA0_N_IRQHandler(){
    uint32_t status  = TA0IV;
    switch (status){
    case 2: {
        TimeInterval = TA0CCR1;
        TA0R = 0;
        if (x == 0) {
            x = 1;
            break;
        }
        if ( x == 1 ) {
            if ((TimeInterval >= STARTBITMIN) && ( TimeInterval <= STARTBITMAX)) {
                x = 2;
                break;
            }
            else {
                x = 0;
                break;
            }
        }
        if (x == 2) {
            if ((TimeInterval >= PAUSEBITMIN) && ( TimeInterval <= PAUSEBITMAX)) {
                x = 3;
                break;
            }
            else {
                x = 0;
                IRready = IRREPEAT;
                break;
            }
        }
        if (( x & 0x01 ) == 0x01 ){
            if ((TimeInterval >= BITMIN) && ( TimeInterval <= BITMAX)) {
                x++ ;
            } else { x = 0; break; }
        }
        else {
            if ((TimeInterval >= ZEROBITMIN) && ( TimeInterval <= ZEROBITMAX)) {
                x++;
                IRdatatmp = (IRdatatmp <<1 );
            }
            else {
                if ((TimeInterval >= ONEBITMIN) && ( TimeInterval <= ONEBITMAX)) {
                    x++;
                    IRdatatmp = (IRdatatmp <<1) + 1;
                }
                else { x=0; break; }
            }
        }
        if (x == 68) {
            x = 0;
            uint16_t address = (IRdatatmp & 0xFFFF0000)>>16;
            uint8_t byte1 = (IRdatatmp & 0x0000FF00)>>8;
            uint8_t byte0 = (IRdatatmp & 0x000000FF);
            if (!(byte1 & byte0)) {
                IRdata = ((IRdatatmp & 0xFF000000)>>16) | ((IRdatatmp & 0x0000FF00)>>8);
                IRaddress = address;
                IRcommand = byte0;
                IRready = IRDONE;
            } else { IRready = IRNONE;}

            __no_operation();
        }
        break;
    }
    case 4: {
        __no_operation();
        TA0R = 0;
        x = 0;
        break;
    }

    case 14: {
        __no_operation();
        TA0R = 0;
        x = 0;
        break;
    }

    default: break;
    }
}
