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
 * 		P1.1, P1.2		Reserved for software UART (Transmit only)
 * 		P1.6, P1.7		USI I2C mode
 * 		P1.3			I2C slave address pin
 * 						High:	0x41 (default)
 * 						Low:	0x43 (= 0x41 | 0x02)
 * 		P1.4			Temperature convert finished interrupt output
 * 		P1.5			Unison alarm interrupt output
 * 		P2.0~P2.5		Individual alarm interrupt output for 6 alarms
 */

#include <msp430.h>
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

const unsigned int _half_second = 16384;		// Half of 1-Hz

// If software UART output enabled
#ifdef _UART_OUTPUT
/**
 * Following value come from sample code of TI
 */
const unsigned int _UART_period_1200 = 0x1B;		// 27, period for generating 1200 baud rate for UART based on 32768-Hz
unsigned char _UART_n_bit;							// Number of bits for each transmit
unsigned int _UART_TX_data;							// Data buffer for UART TX

unsigned char _UART_send = 0;						// Control bit for send data

#endif

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

#ifdef _UART_OUTPUT
    // Setup P1.2 for TXD
    P1SEL |= _UART_TXD;	// P1.2 as TA0.1 out1
    P1DIR |= _UART_TXD; // P1.2 is output pin
#endif

    // Configure for ACLK, no division applied
    BCSCTL3 |= XCAP_3;	// BCSCTL3 |= 0x0C;
    					// XCAPx = 11, Oscillator capacitor ~12.5 pF

    // Setup Timer
    TACTL |= (TASSEL_1 + MC_2);			// TASSELx = 01, using ACLK as source
										// MCx = 02, continuous mode
#ifdef _UART_OUTPUT
    TACTL |= TAIE;						// TAIE, enable overflow interrupt
#endif

    TACCR0 = _half_second;				// The timer clock is 32768-Hz
										// Using 16384 (_half_second) to output full 1-Hz cycle
    TACCTL0 |= CCIE;					// Enable timer capture interrupt

#ifdef _UART_OUTPUT
    // Initialize UART TA0.1
    TACCTL1 = OUT;						// TXD idle at 1.
#endif

    // Initialize data store values
    _init_DS();

    __enable_interrupt();

    while(1) {
#ifdef _UART_OUTPUT
    	if (_UART_send == 2) {
    		_UART_send_datetime();
    		_UART_send = 0;
    	}
#endif
    }
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

#ifdef _UART_OUTPUT
/**
 * Extra functions for software UART
 */
void _UART_TX_byte(unsigned char byte) {
	// Code bases on TI's example
	_UART_n_bit = 0xA;						// Load Bit counter, 8data + ST/SP
	while (TACCR1 != TAR)					// Prevent async capture
		TACCR1 = TAR;						// Current state of TA counter
	TACCR1 += _UART_period_1200;			// Some time till first bit
	_UART_TX_data = byte | 0x100;			// Add mark stop bit to _UART_TX_data
	_UART_TX_data = _UART_TX_data << 1;		// Add space start bit
	TACCTL1 = (OUTMOD0 + CCIE);				// TXD = mark = idle
	while (TACCTL1 & CCIE);					// Wait for TX completion
}

void _UART_send_datetime() {
	unsigned int idx = 8;
	unsigned char byte_h, byte_l;
	while(idx) {
		byte_h = _DATA_STORE[idx - 1] >> 4;
		byte_l = _DATA_STORE[idx - 1] << 4;
		byte_l = byte_l >> 4;
		_UART_TX_byte(byte_h + 0x30);	// The higher 4 bits of 1 byte and converted to numeric ASCII
		_UART_TX_byte(byte_l + 0x30);	// The lower 4 bits of 1 byte and converted to numeric ASCII
		idx--;
	}
	_UART_TX_byte(0x0A);				// Display a new line after each datetime line
}
#endif

/**
 * The timer capture interrupt is set to happen every 0.5s
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void) {
	// Toggle P1.0 output level every 0.5s
	// to form a full 1-Hz square wave output
	P1OUT ^= BIT0;

#ifdef _UART_OUTPUT
	_UART_send++;
#endif

	TACCR0 += _half_second;
}

#ifdef _UART_OUTPUT
/**
 * Interrupt for UART TX
 */
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A1(void) {
	// We only honor TACCR1 interrupt
	// for UART TX
	if (TAIV == 0x02) {
		TACCR1 += _UART_period_1200;
		// Code bases on TI's example
		if (_UART_n_bit == 0)
	    	TACCTL1 &= ~CCIE;					// All bits TXed, disable interrupt
		else {
			TACCTL1 |= OUTMOD2;					// TX Space
			if (_UART_TX_data & 0x01)
				TACCTL1 &= ~OUTMOD2;			// TX Mark
			_UART_TX_data = _UART_TX_data >> 1;
			_UART_n_bit--;
		}
	}
}
#endif
