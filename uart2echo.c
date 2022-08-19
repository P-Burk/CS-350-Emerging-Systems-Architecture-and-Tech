/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
 *  ======== uart2echo.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

//flash functions used to show initialization of board and for debugging
void initFlashFast() {
    unsigned int startFlash = 10;
    unsigned int i;

    for (i = 0; i < startFlash; i++) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        usleep(50000);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        usleep(50000);
    }
}

void initFlashSlow() {
    unsigned int startFlash = 10;
    unsigned int i;

    for (i = 0; i < startFlash / 5; i++) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        sleep(1);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        sleep(1);
    }
}

//states for the state machine
enum LED_States { LED_Init, LED_waitForChar, LED_1stChar, LED_on2ndChar,
                  LED_off2ndChar, LED_off3rdChar} LED_State;

//state machine function
void stateMachine(char input) {

    //switch statement for handling the changing of states in the state machine
    switch(LED_State) {
        case LED_Init:
            break;

        case LED_waitForChar:
            if (input != 'o' && input != 'O') {
                LED_State = LED_waitForChar;
            }
            else {
                LED_State = LED_1stChar;
            }
            break;

        case LED_1stChar:
            if (input == 'n' || input == 'N') {
                LED_State = LED_on2ndChar;
            }
            else if (input == 'f' || input == 'F') {
                LED_State = LED_off2ndChar;
            }
            else {
                LED_State = LED_waitForChar;
            }
            break;

        case LED_on2ndChar:
            LED_State = LED_waitForChar;
            break;

        case LED_off2ndChar:
            if (input == 'f' || input == 'F') {
                LED_State = LED_off3rdChar;
            }
            else {
                LED_State = LED_waitForChar;
            }
            break;

        case LED_off3rdChar:
            LED_State = LED_waitForChar;
            break;

        default:
            LED_State = LED_Init;
            break;

    }

    //switch statement for handling the setting of variables related to the state
    //of the state machine
    switch(LED_State){
        case LED_Init:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            LED_State = LED_waitForChar;
            break;

        case LED_on2ndChar:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            break;

        case LED_off3rdChar:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            break;

        default:
            break;
    }
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char         input;
    const char   echoPrompt[] = "Echoing characters:\r\n";
    UART2_Handle uart;
    UART2_Params uartParams;
    size_t       bytesRead;
    size_t       bytesWritten = 0;
    uint32_t     status = UART2_STATUS_SUCCESS;

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART where the default read and write mode is BLOCKING */
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL) {
        /* UART2_open() failed */
        while (1);
    }

    /* Turn user LED on and off to indicate successful initialization */
    //slow flash
    initFlashSlow();

    //quick flash
    initFlashFast();

    UART2_write(uart, echoPrompt, 5, &bytesWritten);
    LED_State = LED_Init;

    /* Loop forever echoing */
    while (1) {

        bytesRead = 0;
        while (bytesRead == 0 && LED_State != LED_on2ndChar && LED_State != LED_off3rdChar &&
                LED_State != LED_Init) {
            status = UART2_read(uart, &input, 1, &bytesRead);

            if (status != UART2_STATUS_SUCCESS) {
                 /*UART2_read() failed*/
                 while (1);
            }
        }

        stateMachine(input);

        bytesWritten = 0;
        while (bytesWritten == 0 && LED_State != LED_on2ndChar && LED_State != LED_off3rdChar &&
                LED_State != LED_Init) {
            status = UART2_write(uart, &input, 1, &bytesWritten);

            if (status != UART2_STATUS_SUCCESS) {
                 /*UART2_write() failed*/
                while (1);
            }
        }
    }
}
