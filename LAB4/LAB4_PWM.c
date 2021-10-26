/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
/*******************************************************************************
 * MSP432 Timer_A - Variable PWM
 *
 * Description: In this example, the Timer_A module is used to create a precision
 * PWM with an adjustable duty cycle. The PWM initial period is 80 ms and is
 * output on P2.4. The initial duty cycle of the PWM is 10%, however when the
 * button is pressed on P1.1 the duty cycle is sequentially increased by 10%.
 * Once the duty cycle reaches 90%, the duty cycle is reset to 10% on the
 * following button press.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P1.1  |<--Toggle Switch
 *            |                  |
 *            |                  |
 *            |            P2.4  |--> Output PWM
 *            |                  |
 *            |                  |
 *
 *******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,  // Set SMCLK source as 3 MHz as default
        TIMER_A_CLOCKSOURCE_DIVIDER_24, // 3 MHz/24 = 125 kHz 
        10000, // Period of 10000, this would ascertain the interrupt occurance. 1 / 125 kHz = 8 us, 8 us * 10000 = 80000 us = 80 ms. Max interrupt time
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        1000 // Pulse to determine the Duty cycle; min interrupt time
};

/**
 * Duty Cycle = PULSE / PERIOD
 * 1000 / 10000 = 10 %
 **/

Timer_A_PWMConfig pwmConfig2 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_24,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_OUTPUTMODE_RESET_SET,
        1000
};


int main(void)
{
    /* Halting the watchdog */
    MAP_WDT_A_holdTimer();

    /* Configuring P4.4 and P4.5 as Output. P2.4 as peripheral output for PWM and P1.1 for button interrupt */
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    /* Configuring Timer_A to have a period of approximately 80ms and an initial duty cycle of 10% of that (1000 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);

    /* Enabling interrupts and starting the watchdog timer */
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();

    /* Sleeping when not in use */
    while (1)
    {
        PCM_gotoLPM0();
    }
}

/* Port1 ISR - This ISR will progressively step up the duty cycle of the PWM on a button press */
void PORT1_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1)
    {

        if(pwmConfig2.dutyCycle == 9000)
            pwmConfig2.dutyCycle = 1000;

        else
            pwmConfig2.dutyCycle += 1000;

        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);
    }

    if (status & GPIO_PIN4)
        {
            if(pwmConfig.dutyCycle == 9000)
                pwmConfig.dutyCycle = 1000;

            else
                pwmConfig.dutyCycle += 1000;

            Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
        }
}
