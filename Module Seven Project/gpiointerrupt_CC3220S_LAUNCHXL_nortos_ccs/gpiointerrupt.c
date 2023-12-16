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
 *
 *  This program is designed for a thermostat system using the TI SimpleLink CC3220S platform.
 *  It demonstrates the integration and use of various peripherals including GPIO, Timer, I2C,
 *  and UART to create a functional thermostat.
 *
 *  Functionality:
 *  - The program reads room temperature using an I2C-based temperature sensor.
 *  - It controls an LED to simulate a heating system. The LED turns on or off based on the
 *    comparison between the current room temperature and a user-defined setpoint.
 *  - User input is handled through buttons, allowing the increase or decrease of the temperature setpoint.
 *  - The system's status (current temperature, setpoint, heating state, and uptime in seconds)
 *    is communicated over UART, simulating data transmission to a server.
 *  - A state machine approach is used to manage the scheduling of tasks such as reading
 *    temperature, checking button states, and updating the display.
 *
 *  The code is structured into modular functions for each peripheral's initialization and
 *  specific tasks like handling button presses, reading temperature, and updating the display.
 *  The main loop controls the state machine, cycling through the states to perform each task
 *  in a timely manner.
 *
 *  Author: Blake Kemp
 *  Date: December 6th 2023
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// UART display macro
#define DISPLAY(x) UART_write(uart, &output, x)

// Constants for timer periods in milliseconds
#define TIMER_PERIOD 100

/* Function Prototypes */
void initUART(void);
void initI2C(void);
int16_t readTemp(void);
void initTimer(void);
void gpioButtonFxn0(uint_least8_t index);
void gpioButtonFxn1(uint_least8_t index);
void timerCallback(Timer_Handle myHandle, int_fast16_t status);

// Global variables for button states, UART output, and driver handles
int leftButton = 0; // State of left button
int rightButton = 0; // State of right button
char output[64]; // Buffer for UART output
UART_Handle uart; // Handle for the UART
I2C_Handle i2c; // Handle for the I2C
Timer_Handle timer0; // Handle for the Timer
volatile unsigned char TimerFlag = 0; // Flag set by timer callback

// GPIO interrupt callback for left button
void gpioButtonFxn0(uint_least8_t index) {
    leftButton = 1;
}

// GPIO interrupt callback for right button
void gpioButtonFxn1(uint_least8_t index) {
    rightButton = 1;
}

/*
 *  ======== UART Initialization ========
 */
void initUART(void) {
    UART_Params uartParams;
    UART_init();
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        while (1); // UART initialization failed
    }
}

/*
 *  ======== I2C code begin ========
 */
// I2C Global Variables
static const struct
{
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] =
{
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles
I2C_Handle i2c;


void initI2C(void)
{
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1); // I2C initialization failed
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Detecting the type of temperature sensor connected
    bool found = false;
    int8_t i;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    for (i = 0; i < 3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if (!found)
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress));
        // Configure the transaction for temperature reading
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 2;
        txBuffer[0] = sensors[i].resultReg;
    }
}

/*
 *  ======== Temperature Reading Function ========
 */
int16_t readTemp(void)
{
        int16_t temperature = 0;

        if (I2C_transfer(i2c, &i2cTransaction)) {
            // Extract degrees C from the received data
            temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
            temperature *= 0.0078125;

            // If the MSB is set '1', then we have a 2's complement negative value
            if (rxBuffer[0] & 0x80) {
                temperature |= 0xF000;
            }
        } else {
            DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        }

        return temperature;
    }
/*
 *  ======== Timer Initialization and Callback ========
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}
void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.periodUnits = Timer_PERIOD_US;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL)
    {
        /* Failed to initialized timer */
        while (1){}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR)
    {
        /* Failed to start timer */
        while (1){}
    }
}

/* Define states for the state machine */
typedef enum {
    CHECK_BUTTONS,
    UPDATE_TEMPERATURE,
    UPDATE_DISPLAY,
    WAIT_FOR_TIMER
} AppState;

/* Function Prototypes */
void handleButtonPresses(int* setpoint, int* rightButton, int* leftButton);
void updateTemperature(int* temperature, int* heat, int setpoint);
void updateDisplay(int temperature, int setpoint, int heat, int* seconds);

void handleButtonPresses(int* setpoint, int* rightButton, int* leftButton) {
    // Handling button presses to adjust the setpoint temperature
    if (*rightButton) {
        (*setpoint)++;
        *rightButton = 0;
    }
    if (*leftButton) {
        (*setpoint)--;
        *leftButton = 0;
    }
}

void updateTemperature(int* temperature, int* heat, int setpoint) {
    // Reading and updating the temperature, and adjusting the heat accordingly
    *temperature = readTemp();
    *heat = *temperature < setpoint;
    GPIO_write(CONFIG_GPIO_LED_0, *heat ? CONFIG_GPIO_LED_ON : CONFIG_GPIO_LED_OFF);
}

void updateDisplay(int temperature, int setpoint, int heat, int* seconds) {
    // Updating the display with the current status
    DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, (*seconds)++));
}

/*
 *  ======== mainThread ========
 *  Main thread for handling the state machine logic
 */
void *mainThread(void *arg0)
{
    /* Initialize drivers and variables */
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    // Configuring GPIOs for LED and buttons
    /* Configure the red LED */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Configure BUTTON0 pin */
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    // Additional button configuration if available
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

        /* Enable interrupts */
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    /* Task Scheduler Variables */
    unsigned long buttonCheckTime = 0;
    unsigned long tempCheckTime = 0;
    unsigned long displayCheckTime = 0;
    const unsigned long buttonCheckPeriod = 200; // 200ms
    const unsigned long tempCheckPeriod = 500; // 500ms
    const unsigned long displayCheckPeriod = 1000; // 1000ms
    const unsigned long timerPeriod = 100; // 100ms

    int setpoint = 25; // Thermostat setpoint
    int temperature = 0; // Current temperature
    int heat = 0; // Heating state
    int seconds = 0; // Elapsed seconds

    while (1) {
        // Increment the counters for each task. These counters track how much time has passed.
        buttonCheckTime += timerPeriod;
        tempCheckTime += timerPeriod;
        displayCheckTime += timerPeriod;

        if (TimerFlag) {
            TimerFlag = 0;

            // Button Check Task
            if (buttonCheckTime >= buttonCheckPeriod) {
                setpoint += rightButton - leftButton; // Increment setpoint if right button pressed, decrement if left button pressed.
                // Reset button states to avoid repeated counting.
                rightButton = leftButton = 0;
                buttonCheckTime = 0; // Reset the button check timer.
            }

            // Temperature Check Task
            if (tempCheckTime >= tempCheckPeriod) {
                temperature = readTemp();
                // Determine whether to turn on the heat based on the setpoint and current temperature.
                heat = temperature < setpoint ? 1 : 0;
                // Update the state of the LED to reflect the heating status.
                GPIO_write(CONFIG_GPIO_LED_0, heat ? CONFIG_GPIO_LED_ON : CONFIG_GPIO_LED_OFF);
                tempCheckTime = 0; // Reset the temperature check timer.
            }

            // Display Update Task
            if (displayCheckTime >= displayCheckPeriod) {
                // Format and send the current status (temperature, setpoint, heat, time) over UART.
                snprintf(output, sizeof(output), "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds++);
                UART_write(uart, output, strlen(output));
                displayCheckTime = 0; // Reset the display check timer.
            }
        }
    }
}
