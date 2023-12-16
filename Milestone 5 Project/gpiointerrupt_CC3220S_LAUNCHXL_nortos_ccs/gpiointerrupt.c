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
 * This code implements a Morse code message display system using two LEDs
 * (red and green) to represent "SOS" and "OK" messages. The message to be
 * displayed can be toggled between SOS and OK using two buttons. The LEDs
 * display dots and dashes for Morse code, with appropriate timing.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define OFF     0
#define RED     1
#define GREEN   2
#define SOS     0
#define OK      1

unsigned char i = 0;
unsigned char state;
unsigned char MESSAGE_STATE = SOS;
unsigned char BUTTON_STATE = SOS;


// Arrays to tell the state machine which LEDs to turn on/off for SOS and OK messages
int SOS_LEDs[] = {
    // S: ... (dot-dot-dot)
    RED, OFF, RED, OFF, RED, OFF,
    // Space between letters (---)
    OFF, OFF, OFF,
    // O: --- (dash-dash-dash)
    GREEN, GREEN, GREEN, OFF, GREEN, GREEN, GREEN, OFF, GREEN, GREEN, GREEN,
    // Space between letters (---)
    OFF, OFF, OFF,
    // S: ... (dot-dot-dot)
    RED, OFF, RED, OFF, RED, OFF,
    // Space between words (-------)
    OFF, OFF, OFF, OFF, OFF, OFF, OFF,
};

int OK_LEDs[] = {
    // O: --- (dash-dash-dash)
    GREEN, GREEN, GREEN, OFF, GREEN, GREEN, GREEN, OFF, GREEN, GREEN, GREEN,
    // Space between letters (---)
    OFF, OFF, OFF,
    // K: -.- (dash-dot-dash)
    GREEN, GREEN, GREEN, OFF, RED, OFF, GREEN, GREEN, GREEN,
    // Space between words (-------)
    OFF, OFF, OFF, OFF, OFF, OFF, OFF,
};

// Function to handle the timer callback for LED control
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    int *message_leds = (MESSAGE_STATE == SOS) ? SOS_LEDs : OK_LEDs;
    state = message_leds[i];
    switch (state) {
        case OFF:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            break;
        case RED:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            break;
        case GREEN:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            break;
    }
    i++;



    if (i >= sizeof(SOS_LEDs) / sizeof(SOS_LEDs[0])) {
        // If the message is completed, and the button state changed,
        // switch to the new message state and reset the index
        if (BUTTON_STATE != MESSAGE_STATE) {
            MESSAGE_STATE = BUTTON_STATE;
            i = 0;
        }
    }
}

void initTimer(void) {
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000; // Set the timer period to 0.5 seconds (500000us)
    params.periodUnits = Timer_PERIOD_US; // Specify the period units as microseconds
    params.timerMode = Timer_CONTINUOUS_CALLBACK; // Use continuous callback mode
    params.timerCallback = timerCallback; // Set the callback function
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL || Timer_start(timer0) == Timer_STATUS_ERROR) {
        while (1) {} // Handle initialization failure
    }
}

void gpioButtonFxn0(uint_least8_t index) {
    BUTTON_STATE = (BUTTON_STATE == SOS) ? OK : SOS; // Toggle BUTTON_STATE between SOS and OK
}

void gpioButtonFxn1(uint_least8_t index) {
    BUTTON_STATE = (BUTTON_STATE == SOS) ? OK : SOS; // Toggle BUTTON_STATE between SOS and OK
}
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0) {
    // Initialize GPIO, timer, and LEDs
    GPIO_init();
    initTimer();
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);

    // Initialize button 0 and set its callback
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    // Initialize button 1 and set its callback
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    // If button 0 is not the same as button 1, set its callback and enable interrupts
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    }

    while (1) {
        // Loop indefinitely
    }
}
