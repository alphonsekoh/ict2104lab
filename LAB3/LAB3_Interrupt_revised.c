///***************************************************************************************
//  Toggle LED (P1.0)
//  Description; Toggle P1.0 inside of a software loop.
//
//                MSP432P4xx
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.0|-->LED
//

//***************************************************************************************

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>


void main(void)
 {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;       // Stopping the Watchdog Timer

    uint32_t count = 0;                               // Simple counter variable
    int color = 0;

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);      // Set S1 as input

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);      // Set S2 as input
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);


    /**Interrupts Setting**/
    GPIO_interruptEdgeSelect (GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION );
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4 );   //  Clear the interrupt flag for pin P1.1
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);  // Enable interrupt for pin P1.1
    Interrupt_enableInterrupt(INT_PORT1);    // Enable interrupt for Port 2
    Interrupt_enableMaster(); // Enable master interrupt

    while(1)
    {
        PCM_gotoLPM3();  // Forever loop that puts the device to low power mode 3 state.

    }
}

void PORT1_IRQHandler(void){
    volatile uint32_t i;
    volatile static uint32_t counter;
    if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == ((uint8_t)0x00)){
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    }

    if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == ((uint8_t)0x00)){
        if (counter % 3 == 0){
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        }
        else if (counter % 3 == 1){
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        }
        else if (counter % 3 == 2){
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
        }
        GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
        counter += 1;
    }

    // Debouncing loop
    for(i = 0; i < 5000; i++);

    SCB->SCR&= ~SCB_SCR_SLEEPONEXIT_Msk;    // Disable SLEEPON EXIT
    __DSB();
}
