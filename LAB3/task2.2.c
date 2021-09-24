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
/******************************************************************************
 * MSP432 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 * Author: 
*******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

int counter;
int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    counter = 0;
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN5); // Configure pin P2.5 as input (with pull up resistor)
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);  // Configure pin P1.0 as output
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    /** Interrupt GPIO Setting **/
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN5);   //  Clear the interrupt flag for pin P2.5
    MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN5);  // Enable interrupt for pin P2.5
    Interrupt_enableInterrupt(INT_PORT2);    // Enable interrupt for Port 2
    Interrupt_enableMaster(); // Enable master interrupt

    while(1){
      PCM_gotoLPM3();  // Forever loop that puts the device to low power mode 3 state.
    }

}

void PORT2_IRQHandler(void){
    uint_fast16_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);  //interrupt status for Port 2 and store it into the local vari


    counter++;

    if(counter == 20){
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0); // Check if the global variable is equal to 20 (number of notches on the wheel encoder).
        counter = 0;
    }
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);
}

