int main(void)
{
    P1->SEL0 = 0;
    P1->SEL1 = 0;
    P1->DIR |= BIT0;            // LED1
    P1->DIR &= ~(BIT1 | BIT4);  // Switch S1 & S2
    P1->REN |= (BIT1 | BIT4);   // Set pull-up resistor
    P1->OUT |= (BIT1 | BIT4);   // Enable internal resistor

    P1->IES |= (BIT1 | BIT4);   // Interrupt on high-to-low transition
    P1->IFG = 0;                // Clear all P1 interrupt flags
    P1->IE |= (BIT1 | BIT4);    // Enable interrupt for P1.1 & P1.4

    // Enable Port 1 interrupt on the NVIC
    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);

    P2->SEL0 = 0;
    P2->SEL1 = 0;
    P2->DIR |= (BIT0 | BIT1 | BIT2);    // LED2
}

/* Port1 ISR */
void PORT1_IRQHandler(void)
{
    volatile uint32_t i;
    volatile static uint32_t counter = 0;

    // Toggling the output on the LED
    if(P1->IFG & BIT1)                  // P1.1 = S1 switch
    {
        P1OUT ^= BIT0;                  // P1.0 = toggle LED1
        P1->IFG &= ~BIT1;
    }

    if(P1->IFG & BIT4)                  // P1.4 = S2 Switch
    {
        if ((BIT0 & P1IN) == BIT0)      // if RED LED is off
        {
            if ((P2IN & 0x07) == 0x07)  // WHITE
                P2OUT &= ~0x07;         // OFF ALL LEDs
            else
                P2OUT += 1;             // change change
        }
        else
        {
            P2OUT &= ~0x07;             // OFF ALL LEDs
            if(counter%3 == 0 )
                P2OUT |= BIT0;          // RED
            else if(counter%3 == 1)
                P2OUT |= BIT1;          // GREEN
            else if(counter%3 == 2)
                P2OUT |= BIT2;          // BLUE
        }
        P1->IFG &= ~BIT4;
    }
    counter += 1;

    // Delay for switch debounce
    for(i = 0; i < 5000; i++);

    SCB->SCR&= ~SCB_SCR_SLEEPONEXIT_Msk;    // Disable SLEEPON EXIT
    __DSB();                                // Ensures SLEEPONEXIT is set immediately before exiting ISR
}

