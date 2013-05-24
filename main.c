#include <msp430g2452.h>

/*
 * main.c
 */
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    // Set MCLK and SMCLK
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    // Configure for ACLK
    //BCSCTL3 |= 0x0C; // XCAPx = 11, Oscillator capacitor ~12.5 pF
    BCSCTL3 |= XCAP_3;
    //BCSCTL3 &= 0xCF; // LFXT1Sx = 00(Default), 32768-Hz crystal on LFXT1
    //BCSCTL1 &= ~XTS; // XTS = 0(Default), LFXT1 in Low-frequency mode
    //BCSCTL1 |= 0x30; // DIVAx = 11, ACLK / 8
    BCSCTL1 |= DIVA_3;

    // Clear LFXT1 fault flag
    //BCSCTL3 &= ~LFXT1OF;

    // Set P1.0 as TA0CLK
    P1DIR |= (BIT0 + BIT6); // P1.0, P1.6 as output
    //P1SEL |= BIT0; // Using TA0CLK on P1.0

    P1OUT |= BIT0; // P1.0 as high
    P1OUT &= ~BIT6; // P1.6 as low

    // Setup Timer
    //TACTL |= 0x0100u; // TASSELx = 01, using ACLK as source
    //TACTL |= 0x00C0u; // IDx = 11, devide by 8
    //TACTL |= 0x0010u; // MCx = 01, up mode
    TACTL |= (TASSEL_1 + ID_3 + MC_1);

    TACCR0 = 256;	// The timer clock is 32768-Hz / 8 / 8 = 512
    				// Using 256 to output 2-Hz

    //TACCTL0 |= 0x0080u; // OUTMODx = 100
    TACCTL0 |= (OUTMOD2 + CCIE);

    __enable_interrupt();

    while(1);
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void) {
	P1OUT ^= BIT0;
	P1OUT ^= BIT6;

	TACCTL0 &= ~CCIFG;
}
