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
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);

/*****************************
 ** Helper Global Variables **
 *****************************/
volatile unsigned char timerFlag = 0;
volatile unsigned char msgFlag = 0;
volatile unsigned char currTempFlag = 0;
volatile int16_t updateTemp = 0;
volatile int16_t currTemp = 0;
volatile int16_t setTemp = 30;
volatile unsigned char heaterOn = 0;
volatile unsigned int secCount = 0;

//LED flash functions for debugging
void initFlashFast();
void initFlashSlow();
void quickFlash(unsigned int flashNum);

//declare state machines
void tscStateMachine();
void spaStateMachine();
void hcStateMachine();
void srStateMachine();

/**************************
 ** I2C Global Variables **
 **************************/
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

/*******************************************
 ** I2C Driver Handles - Global variables **
 *******************************************/
I2C_Handle i2c;

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

/***************************
 ** UART Global Variables **
 ***************************/
char output[64];
int bytesToSend;
const char   echoPrompt[] = "Echoing characters:\r\n";
size_t       bytesWritten = 0;  //added

/********************************************
 ** UART Driver Handles - Global variables **
 ********************************************/
UART_Handle uart;

/****************************
 ** Timer Global Variables **
 ****************************/
Timer_Handle timer0;


/************************************************************************
 ***************************** I2C SECTION ******************************
 ************************************************************************/

/***********************
 ** I2C Init Function **
 ***********************/
// Make sure you call initUART() before calling this function.
void initI2C(void) {
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: "
                "%x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    } else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, "
                "contact professor\n\r"))
    }
}

/**************************************
 ** I2C Temperature Reading Function **
 **************************************/
int16_t readTemp(void) {
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    } else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r \n\r"))
    }
    return temperature;
}



/************************************************************************
 ***************************** UART SECTION *****************************
 ************************************************************************/

/************************
 ** UART Init Function **
 ************************/
void initUART(void) {
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}



/************************************************************************
 ***************************** TIMER SECTION ****************************
 ************************************************************************/

/***********************************
 ** Timer Callback Function (ISR) **
 ***********************************/
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    spaStateMachine();
    tscStateMachine();
    hcStateMachine();
    srStateMachine();
    timerFlag++;
}

/*************************
 ** Timer Init Function **
 *************************/
void initTimer(void) {
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
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



/************************************************************************
 *************************** HELPER FUNCTIONS ***************************
 ************************************************************************/

// used to signal initialization of board/start of main()
void initFlashFast() {
    unsigned int startFlash = 10;
    unsigned int i;

    for (i = 0; i < startFlash; i++) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        usleep(50000);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        usleep(50000);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }
}

// used to signal initialization of board/start of main()
void initFlashSlow() {
    unsigned int startFlash = 10;
    unsigned int i;

    for (i = 0; i < startFlash / 5; i++) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        sleep(1);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        sleep(1);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }
}

// function for flashing the red LED quickly
// used for debugging
void quickFlash(unsigned int flashNum) {
    unsigned int i;

    for (i = 0; i < flashNum; i++) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        usleep(50000);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        usleep(50000);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }
}



/************************************************************************
 **************************** STATE MACHINES ****************************
 ************************************************************************/

/**********************************
 ** Temperature Set Point Adjust **
 **********************************/
//states
enum SPA_States { spInit, spWait, spUpdate } SPA_State;

//state machine
void spaStateMachine() {

    //switch statement for handling the transitions of states
    switch(SPA_State) {
        case spInit:
            setTemp = 30;
            updateTemp = 0;
            SPA_State = spWait;
            break;

        case spWait:
            if (timerFlag % 2 == 0) {
                SPA_State = spUpdate;
            }
            break;

        case spUpdate:
            SPA_State = spWait;
            break;

        default:
            SPA_State = spInit;
            break;
    }

    //switch statement for handling actions of states
    switch(SPA_State) {
        case spInit:
            break;

        case spWait:
            break;

        case spUpdate:
            setTemp += updateTemp;
            updateTemp = 0;
            break;

        default:
            SPA_State = spInit;
            break;
    }
}

/******************************
 ** Temperature Sensor Check **
 ******************************/
//states
enum TSC_States { tscInit, tscWait, tscUpdate } TSC_State;

//state machine
void tscStateMachine() {

    //switch statement for handling the transitions of states
    switch(TSC_State) {
        case tscInit:
            currTempFlag = 0;
            TSC_State = tscWait;
            break;

        case tscWait:
            if (timerFlag % 5 == 0) {
                TSC_State = tscUpdate;
            }
            break;

        case tscUpdate:
            TSC_State = tscWait;
            break;

        default:
            TSC_State = tscInit;
            break;
    }

    //switch statement for handling actions of states
    switch(TSC_State) {
        case tscInit:
            break;

        case tscWait:
            break;

        case tscUpdate:
            //currTemp = readTemp();
            currTempFlag = 1;
            break;

        default:
            TSC_State = tscInit;
            break;
    }
}

///********************
// ** Heater Control **
// ********************/
//// HC + REPORTING
////states
//enum HC_States { hcInit, hcWait, hcCheck, hcHeaterOn, hcHeaterOff, hcOutput } HC_State;
//
////state machine
//void hcStateMachine() {
//
//    //switch statement for handling the transitions of states
//    switch(HC_State) {
//        case hcInit:
//            heaterOn = 0;
//            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
//            HC_State = hcWait;
//            break;
//
//        case hcWait:
//            if (timerFlag % 10 == 0) {
//                HC_State = hcCheck;
//            }
//            break;
//
//        case hcCheck:
//            if (currTemp < setTemp) {
//                HC_State = hcHeaterOn;
//            } else {
//                HC_State = hcHeaterOff;
//            }
//            break;
//
//        case hcHeaterOn:
//            HC_State = hcOutput;
//            break;
//
//        case hcHeaterOff:
//            HC_State = hcOutput;
//            break;
//
//        case hcOutput:
//            HC_State = hcWait;
//            break;
//
//        default:
//            HC_State = hcInit;
//            break;
//    }
//
//    //switch statement for handling actions of states
//    switch(HC_State) {
//        case hcInit:
//            break;
//
//        case hcWait:
//            break;
//
//        case hcCheck:
//            break;
//
//        case hcHeaterOn:
//            heaterOn = 1;
//            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
//            break;
//
//        case hcHeaterOff:
//            heaterOn = 0;
//            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
//            break;
//
//        case hcOutput:
//            secCount++;
//            msgFlag = 1;
//            //DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", currTemp, setTemp, heaterOn, secCount))
//            break;
//
//        default:
//            HC_State = hcInit;
//            break;
//    }
//}


/********************
 ** Heater Control **
 ********************/
// JUST HC
//states
enum HC_States { hcInit, hcWait, hcCheck, hcHeaterOn, hcHeaterOff } HC_State;

//state machine
void hcStateMachine() {

    //switch statement for handling the transitions of states
    switch(HC_State) {
        case hcInit:
            heaterOn = 0;
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            HC_State = hcWait;
            break;

        case hcWait:
            if (timerFlag % 10 == 0) {
                HC_State = hcCheck;
            }
            break;

        case hcCheck:
            if (currTemp < setTemp) {
                HC_State = hcHeaterOn;
            } else {
                HC_State = hcHeaterOff;
            }
            break;

        case hcHeaterOn:
            HC_State = hcWait;
            break;

        case hcHeaterOff:
            HC_State = hcWait;
            break;

        default:
            HC_State = hcInit;
            break;
    }

    //switch statement for handling actions of states
    switch(HC_State) {
        case hcInit:
            break;

        case hcWait:
            break;

        case hcCheck:
            break;

        case hcHeaterOn:
            heaterOn = 1;
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            break;

        case hcHeaterOff:
            heaterOn = 0;
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            break;

        default:
            HC_State = hcInit;
            break;
    }
}


/*******************
 ** Status Report **
 *******************/
//states
enum SR_States { srInit, srWait, srOutput } SR_State;

//state machine
void srStateMachine() {

    //switch statement for handling the transitions of states
    switch(SR_State) {
        case srInit:
            SR_State = srWait;
            break;

        case srWait:
            if (timerFlag % 10 == 0) {
                SR_State = srOutput;
            }
            break;

        case srOutput:
            SR_State = srWait;
            break;

        default:
            SR_State = srInit;
            break;
    }

    //switch statement for handling actions of states
    switch(SR_State) {
        case srInit:
            break;

        case srWait:
            break;

        case srOutput:
            secCount++;
            msgFlag = 1;
            break;

        default:
            SR_State = srInit;
            break;
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index) {
    updateTemp--;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index) {
    updateTemp++;
}




/*
 *  ======== mainThread ========
 */

void *mainThread(void *arg0)
{
    initFlashSlow();
    initFlashFast();
    sleep(0.5);

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
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

    //instantiate the components
    initUART();
    initI2C();
    initTimer();

    //set initial states of state machines
    SPA_State = spInit;
    TSC_State = tscInit;
    HC_State = hcInit;
    SR_State = srInit;


    while (1) {

        //set current temp when temp flag is raised
        if (currTempFlag == 1) {
            currTemp = readTemp();
            currTempFlag = 0;
        }

        //send message to server when msg flag is raised
        if (msgFlag == 1) {
            DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", currTemp, setTemp, heaterOn, secCount))
            msgFlag = 0;
            timerFlag = 0;      //reset timerFlag because sending message has the longest period
        }
    }


    return (NULL);
}
