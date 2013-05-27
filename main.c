/**
 * RTC and Temperature Source using MSP430G2452
 *
 * HouYu Li <karajan_ii@hotmail.com>
 *
 * No license applied. Use as you wish.
 *
 * Data structure following DS3231.
 *
 * Port definition
 * 		P1.0			1-Hz output
 * 		P1.1, P1.2		Reserved for software UART (DEBUG)
 * 		P1.6, P1.7		USI I2C mode
 * 		P1.3			I2C slave address pin
 * 						High:	0x41 (default)
 * 						Low:	0x43 (= 0x41 | 0x02)
 * 		P1.4			Temperature convert finished interrupt output
 * 		P1.5			Unison alarm interrupt output
 * 		P2.0~P2.5		Individual alarm interrupt output for 6 alarms
 */

#include <msp430g2452.h>
#include "config.h"
#include "functions.h"

unsigned char _DATA_STORE[31];	// Data storage
								// 0: RTC second in BCD
								// 1: RTC minute in BCD
								// 2: RTC hour in BCD 24-hour format
								// 3: RTC day in BCD. 1~7: Mon~Sun
								// 4: RTC date in BCD
								// 5: RTC month in BCD
								// 6: RTC year in BCD
								// 7: RTC century in BCD
								// 8~10: Alarm1: minute(BCD), hour(BCD), day(s)(Bit Mask)
									// MSB of each byte sets the match action
								// 11~25: Same as 8~10 for Alarm2~Alarm6
								// 26: MSB of temperature
								// 27: LSB of temperature
								// 28: Reserved for general configuration
								// 29: Alarm interrupt enable bits
									// MSB sets the way in which the MCU output the interrupt
										// 1: All alarm interrupt output on P1.5
										// 0: Each alarm interrupt output is mapped to P2.0~P2.5
								// 30: Alarm interrupt flags
/*
 * main.c
 */
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    // Set MCLK and SMCLK
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    // Set P1.0 to output 1-Hz
    P1DIR |= BIT0;		// P1.0 as output
    P1OUT &= ~BIT0;		// P1.0 low initially
    // Setup P1.3 for address selection
    P1REN |= BIT3;		// Enable pull resistors on P1.3
    P1OUT |= BIT3;		// Using pull-up resistor on P1.3

    // Configure for ACLK
    BCSCTL3 |= XCAP_3;	// BCSCTL3 |= 0x0C;
    					// XCAPx = 11, Oscillator capacitor ~12.5 pF
    BCSCTL1 |= DIVA_3;	// BCSCTL1 |= 0x30;
    					// DIVAx = 11, ACLK = LFXT1 / 8
    //BCSCTL1 &= ~XTS;	// XTS = 0(Default), LFXT1 in Low-frequency mode
    //BCSCTL3 &= 0xCF;	// LFXT1Sx = 00(Default), 32768-Hz crystal on LFXT1

    // Setup Timer
    TACTL |= (TASSEL_1 + ID_3 + MC_1);	// TACTL |= (0x0100u + 0x00C0u + 0x0010u);
										// TASSELx = 01, using ACLK as source
										// IDx = 11, timer clock = ACLK / 8
										// MCx = 01, up mode
    TACCR0 = 256;						// The timer clock is 32768-Hz / 8 / 8 = 512
										// Using 256 to output full 1-Hz cycle
    TACCTL0 |= CCIE;					// Enable timer capture interrupt

    // Initialize data store values
    _init_DS();

    __enable_interrupt();
}

/**
 * Extra functions
 */

/**
 * Initialize data store values
 */
void _init_DS() {
	// The default RTC time is 2000-1-1 00:00:00, Saturday
	_DATA_STORE[3] = 0x06; // Day = 6, Saturday
	_DATA_STORE[4] = 0x01; // Date = 1
	_DATA_STORE[5] = 0x01; // Month = 1
	_DATA_STORE[7] = 0x20; // Century = 20
}

/**
 * The timer capture interrupt is set to happen every 0.5s
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void) {
	// Toggle P1.0 output level every 0.5s
	// to form a full 1-Hz square wave output
	P1OUT ^= BIT0;

	TACCTL0 &= ~CCIFG;
}
