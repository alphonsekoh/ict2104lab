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

#define WAIT_INIT_MULTI ((uint32_t)(15))                 // Multiplier for longer busy wait
#define WAITCYCLES_BASE ((uint32_t)(10000))              // Base value for the wait cycles calculation
#define WAIT_CYCLES     ((uint32_t)(WAITCYCLES_BASE))    // Constant define for the initial number of waiting cycles
#define REGBASEADR ((uint32_t)(0x40004C00)) // Base addr. of Port 1 configuration reg
#define REG_SEL0  ((uint32_t)(0x0000000A)) // Addr. Offset for SEL 0 reg
#define REG_SEL1 ((uint32_t)(0x0000000C)) // Addr. Offset for SEL 1 reg
#define REG_DIR ((uint32_t)(0x00000004)) // Direction Offset in Port 1 conf
#define REG_OUT ((uint32_t)(0x00000002)) // Output Value Offset in Port 1
#define BIT0 (uint16_t)(0x0001)


// Global variable defining the number of cycles of the busy loop
uint32_t g_waitcycles = WAIT_CYCLES * WAIT_INIT_MULTI;

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;       // Stopping the Watchdog Timer

    uint32_t count = 0;                               // Simple counter variable
    int color = 0;

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);      // Set S1 as input

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);      // Set S2 as input
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);


    while(1)
    {
        if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == ((uint8_t)0x00)){
            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
            for(count = 0; count < g_waitcycles; count++){}   // Busy Loop for Delay

        }
        switch(color){
        case 0:
            /** RED LED **/
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == ((uint8_t)0x00)){
                for(count = 0; count < g_waitcycles; count++){}
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);

            }
                color++;
                break;
        case 1:
            /** GREEN LED **/
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == ((uint8_t)0x00)){
                for(count = 0; count < g_waitcycles; count++){}
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);

            }
                color++;
                break;
        case 2:
            /** BLUE LED **/
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == ((uint8_t)0x00)){
                for(count = 0; count < g_waitcycles; count++){}
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);


            }
                color++;
                break;
        default:
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == ((uint8_t)0x00)){
                for(count = 0; count < g_waitcycles; count++){}
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

            }
            color = 0;

            }


    }
}

