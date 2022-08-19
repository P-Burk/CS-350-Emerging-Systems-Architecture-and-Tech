/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"


volatile unsigned char timerFlag = 1;
volatile unsigned char buttonFlag = 0;
volatile unsigned char msgFlag = 0;

// used to signal initialization of board/start of main()
void initFlashFast() {
    unsigned int startFlash = 10;
    unsigned int i;

    for (i = 0; i < startFlash; i++) {
        GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);
        usleep(50000);
        GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_ON);
        usleep(50000);
        GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);
    }
}

// used to signal initialization of board/start of main()
void initFlashSlow() {
    unsigned int startFlash = 10;
    unsigned int i;

    for (i = 0; i < startFlash / 5; i++) {
        GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);
        sleep(1);
        GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_ON);
        sleep(1);
        GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);
    }
}

// function for flashing the yellow LED quickly
// used for debugging
void quickFlash(unsigned int flashNum) {
    unsigned int i;

    for (i = 0; i < flashNum; i++) {
        GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);
        usleep(50000);
        GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_ON);
        usleep(50000);
        GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);
    }
}

// flash the LED for the first 'S' in SOS
void sFlash(unsigned char timerFlag) {
    if (timerFlag > 5 || timerFlag % 2 == 0) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    }
}

// flash the LED for 'O'
void oFlash(unsigned char timerFlag) {
    if (timerFlag >= 12 || timerFlag % 4 == 0) {
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
    }
}

// flash the LED for the second 'S' in SOS
void sFlash2(unsigned char timerFlag) {
    if (timerFlag % 2 == 0) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    }
}

// flash the LEDs for 'K'
void kFlash(unsigned char timerFlag) {
    if (timerFlag == 4 || timerFlag == 6) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    }
    else if (timerFlag == 5) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
    }
}

//states for the state machine
enum LED_States { LED_Init, LED_checkMsgFlag, LED_sFlash, LED_oFlash, LED_sFlash2, LED_kFlash,
                  LED_wordWait } LED_State;

//state machine function
void stateMachine() {

    //switch statement for handling the transitions of states
    switch(LED_State) {
        case LED_Init:
            break;

        case LED_checkMsgFlag:
            if (msgFlag == 0) {
                timerFlag = 1;
                LED_State = LED_sFlash;
            }
            else {
                timerFlag = 1;
                LED_State = LED_oFlash;
            }
            break;

        case LED_sFlash:
            if (timerFlag == 9) {
                timerFlag = 1;
                LED_State = LED_oFlash;
            }
            break;

        case LED_oFlash:
            if (timerFlag == 16 && msgFlag == 0) {
                timerFlag = 1;
                LED_State = LED_sFlash2;
            }
            else if (timerFlag == 16 && msgFlag == 1) {
                timerFlag = 1;
                LED_State = LED_kFlash;
            }
            break;

        case LED_sFlash2:
            if (timerFlag == 6) {
                timerFlag = 1;
                LED_State = LED_wordWait;
            }
            break;

        case LED_kFlash:
            if (timerFlag == 10) {
                timerFlag = 1;
                LED_State = LED_wordWait;
            }
            break;

        case LED_wordWait:
            if (timerFlag == 8) {
                timerFlag = 1;
                LED_State = LED_checkMsgFlag;
            }
            break;

        default:
            LED_State = LED_Init;
            break;
    }

    //switch statement for handling actions of states
    switch(LED_State){
        case LED_Init:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            timerFlag = 1;
            buttonFlag = 0;
            msgFlag = 0;
            LED_State = LED_checkMsgFlag;
            break;

        case LED_sFlash:
            sFlash(timerFlag);
            break;

        case LED_oFlash:
            oFlash(timerFlag);
            break;

        case LED_sFlash2:
            sFlash2(timerFlag);
            break;

        case LED_kFlash:
            kFlash(timerFlag);
            break;

        case LED_wordWait:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            msgFlag = buttonFlag;
            break;

        default:
            break;
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_1);

    buttonFlag = !buttonFlag;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0);

    buttonFlag = !buttonFlag;
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    //quickFlash(TimerFlag);
    stateMachine();
    timerFlag++;
}

void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
    /* Failed to initialized timer */

        while (1) {}

    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
    /* Failed to start timer */
        while (1) {}
    }
}


/*
 *  ======== mainThread ========
 */

void *mainThread(void *arg0)
{
    initFlashSlow();
    initFlashFast();
    sleep(2);

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    LED_State = LED_Init;
    initTimer();

    return (NULL);
}
