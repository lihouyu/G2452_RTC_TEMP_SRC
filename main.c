/**
 * RTC and Temperature Source using MSP430G2452
 *
 * HouYu Li <karajan_ii@hotmail.com>
 *
 * No license applied. Use as you wish.
 *
 * Data structure is following DS3231.
 *
 * Port definition
 *      P1.0            1-Hz output
 *      P1.1, P1.2      Reserved for software UART (Transmit only)
 *      P1.3            I2C slave address pin
 *                      High:   0x41 (default)
 *                      Low:    0x43 (= 0x41 | 0x02)
 *      P1.4            Temperature convert finished interrupt output
 *      P1.5            Unison alarm interrupt output for all 6 alarms
 *      P1.6, P1.7      USI I2C mode
 *      P2.0            Individual alarm interrupt output for Alarm1
 *      P2.1            Individual alarm interrupt output for Alarm2
 *      P2.2            Individual alarm interrupt output for Alarm3
 *      P2.3            Active low logic for setting CPU speed at 8MHz
 *      P2.4            Active low logic for setting CPU speed at 12MHz
 *      P2.5            Active low logic for setting CPU speed at 16MHz
 */

#include <msp430.h>

#include "config.h"
#include "functions.h"
#include "USI_I2C_slave.h"

unsigned char _DATA_STORE[31];  // Data storage
                                // 0: RTC second in BCD
                                // 1: RTC minute in BCD
                                // 2: RTC hour in BCD 24-hour format
                                // 3: RTC day in BCD. 1~7: Mon~Sun
                                // 4: RTC date in BCD
                                // 5: RTC month in BCD
                                // 6: RTC year in BCD
                                // 7: RTC century in BCD
                                // 8~10: Alarm1: minute(BCD), hour(BCD), day(s)(Bit Mask)
                                    // MSB of byte 9 is the match enable bit
                                // 11~25: Same as 8~10 for Alarm2~Alarm6
                                // 26: High parts of temperature
                                // 27: Low parts of temperature
                                // 28: Reserved for general configuration
                                    // BIT7: Dedicated interrupt output for Alarm1~3
                                    // BIT6: Start temperature convert bit
                                    // BIT5: Temperature convert finished flag
                                // 29: Alarm interrupt enable bits
                                // 30: Alarm interrupt flags

const unsigned int _second_div = 2048;      // 1/16 of 1-Hz with a bit tuning
unsigned int _second_tick = 0;              // Ticker for a second
unsigned char _is_leap_year = 0;            // Leap year indicator

unsigned char _I2C_data_offset = 0;         // Offset for data accessing in I2C

unsigned char _RTC_action_bits = 0x00;      // For marking actions in interrupt
                                            // and run the action in the main loop
unsigned char _RTC_action_bits2 = 0x00;     // The extended action bits

unsigned char _RTC_byte_l = 0, _RTC_byte_h = 0; // For calculation use
unsigned int _TEMP_data = 0;                    // For holder temperature result data
unsigned char _TEMP_data_user_read = 0;         // Temperature data read by user

/***********************************************
 * Callback related variables (Mandatory)
 * Do not change the variable name
 ***********************************************/
unsigned char _USI_I2C_slave_n_byte = 0;
//**********************************************/

// If software UART output enabled
#ifdef _UART_OUTPUT
/**
 * Following value come from sample code of TI
 */
const unsigned int _UART_period_1200 = 0x1B;    // 27, period for generating 1200 baud rate for UART based on 32768-Hz
unsigned char _UART_n_bit;                      // Number of bits for each transmit
unsigned int _UART_TX_data;                     // Data buffer for UART TX
#endif

/*
 * main.c
 */
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    // Set MCLK and SMCLK
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    // Set P1.0 to output 1-Hz
    P1DIR |= BIT0;              // P1.0 as output
    P1OUT &= ~BIT0;             // P1.0 is low initially
    // Setup P1.3 for address selection
    P1REN |= BIT3;              // Enable pull resistor on P1.3
    P1OUT |= BIT3;              // Using pull-up resistor on P1.3
    // Setup P2.3, P2.4, P2.5 for CPU speed selection
    P2REN |= (BIT3 + BIT4 + BIT5);  // Enable pull resistors
    P2OUT |= (BIT3 + BIT4 + BIT5);  // Using pull-up resistors
    // Setup pins for alarm interrupt output
    P1DIR |= (BIT4 + BIT5);         // Set P1.5 as unison alarm interrupt output pin
                                    // Set P1.4 as output for temperature convert finish interrupt
    P1OUT &= ~(BIT4 + BIT5);        // Set P1.4, P1.5 low by default
    P2DIR |= (BIT0 + BIT1 + BIT2);  // Set P2.0~P2.2 as dedicated output for alarm1~3
    P2OUT &= ~(BIT0 + BIT1 + BIT2); // Default low

    // Set CPU speed based on P2 connection
    if (!(P2IN & BIT3)) {           // P2.3 low, set CPU speed at 8MHz
        BCSCTL1 = CALBC1_8MHZ;
        DCOCTL = CALDCO_8MHZ;
    }
    if (!(P2IN & BIT4)) {           // P2.4 low, set CPU speed at 12MHz
        BCSCTL1 = CALBC1_12MHZ;
        DCOCTL = CALDCO_12MHZ;
    }
    if (!(P2IN & BIT5)) {           // P2.5 low, set CPU speed at 16MHz
        BCSCTL1 = CALBC1_16MHZ;
        DCOCTL = CALDCO_16MHZ;
    }

#ifdef _UART_OUTPUT
    // Setup P1.2 for TXD
    P1SEL |= _UART_TXD;         // P1.2 as TA0.1 out1
    P1DIR |= _UART_TXD;         // P1.2 is output pin
#endif

    // Configure for ACLK, no division applied
    BCSCTL3 |= XCAP_3;          // BCSCTL3 |= 0x0C;
                                // XCAPx = 11, Oscillator capacitor ~12.5 pF

    // Setup Timer
    TACTL |= (TASSEL_1 + MC_2); // TASSELx = 01, using ACLK as source
                                // MCx = 02, continuous mode
#ifdef _UART_OUTPUT
    TACTL |= TAIE;              // TAIE, enable overflow interrupt
#endif

    TACCR0 = _second_div;       // The timer clock is 32768-Hz
                                // _second_div is 32768-Hz / 16
                                // so that we have enough space for doing different actions
    TACCTL0 |= CCIE;            // Enable timer capture interrupt

#ifdef _UART_OUTPUT
    // Initialize UART TA0.1
    TACCTL1 = OUT;              // TXD idle at 1.
#endif

    // Initialize data store values
    _init_DS();
    // Check leap year with initial data
    _check_leap_year();

    // Prepare the ADC10 for temperature
    // From TI's sample code
    ADC10CTL1 = INCH_10 + ADC10DIV_3;
    ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;

    // Start I2C slave
    if (P1IN & BIT3)
        USI_I2C_slave_init(_I2C_addr);
    else
        USI_I2C_slave_init(_I2C_addr_op1);

    __enable_interrupt();

    while(1) {
        if (_RTC_action_bits & BIT0) {  // The main timer increment
            _time_increment();
            _RTC_action_bits &= ~BIT0;
        }
        if (_RTC_action_bits & BIT3) {  // Check alarm logic
            _check_alarms();
            _RTC_action_bits &= ~BIT3;
        }
        if (_RTC_action_bits & BIT4) {  // Check alarm interrupt
            _alarm_interrupt();
            _RTC_action_bits &= ~BIT4;
        }
        if (_RTC_action_bits & BIT5) {  // Reset alarm interrupt output
            _alarm_reset_interrupt();
            _RTC_action_bits &= ~BIT5;
        }
#ifdef _UART_OUTPUT
        if (_RTC_action_bits & BIT1) {
            _UART_send_datetime();
            _RTC_action_bits &= ~BIT1;
        }
#endif
        if (_DATA_STORE[28] & BIT6) {   // Temperature convert start bit is set
            ADC10CTL0 |= ENC + ADC10SC; // Start convert temperature
            _DATA_STORE[28] &= ~BIT6;   // Clear the start bit
        }
        if (_RTC_action_bits & BIT6) {  // Go on transfer temperature data
            ADC10CTL0 &= ~ENC;          // Manually clear ADC convert bit
            _TEMP_data = ADC10MEM;
            _DATA_STORE[26] = (char)(_TEMP_data >> 8);
            _DATA_STORE[27] = (char)_TEMP_data;
            _DATA_STORE[28] |= BIT5;    // Temperature data ready for access
            _RTC_action_bits &= ~BIT6;
        }
        if (_RTC_action_bits2 & BIT0) {
            if (_DATA_STORE[28] & BIT5) // If temperature data is ready, we trigger interrupt
                P1OUT |= BIT4;
            _RTC_action_bits2 &= ~BIT0;
        }
        if (_RTC_action_bits2 & BIT1) { // Reset temperature finish interrupt output
            P1OUT &= ~BIT4;
            _RTC_action_bits2 &= ~BIT1;
        }
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
    _DATA_STORE[3] = 0x06;  // Day = 6, Saturday
    _DATA_STORE[4] = 0x01;  // Date = 1
    _DATA_STORE[5] = 0x01;  // Month = 1
    _DATA_STORE[7] = 0x20;  // Century = 20
}

/**
 * Check whether current year is leap year
 */
void _check_leap_year() {
    // Reset leap year indicator first
    _is_leap_year = 0;

    _RTC_byte_l = _DATA_STORE[6] << 4;
    if (_DATA_STORE[6] & 0x10) {
        if (_RTC_byte_l == 0x20 || _RTC_byte_l == 0x60)
            _is_leap_year = 1;
    } else {
        if (_RTC_byte_l == 0x40 || _RTC_byte_l == 0x80)
            _is_leap_year = 1;
    }
}

/**
 * Do time increment
 */
void _time_increment() {
    // Do the main time increment logic here
    _DATA_STORE[0]++;
    if (_DATA_STORE[0] == 0x5A) {   // Check second
        _DATA_STORE[0] = 0x00;
        _DATA_STORE[1]++;   // Add 1 minute
        _RTC_action_bits |= BIT3;   // Let's check alarms when second becomes 0
    } else {
        _time_carry(_DATA_STORE);
    }

    if (_DATA_STORE[1] == 0x5A) {   // Check minute, same logic with minute
        _DATA_STORE[1] = 0x00;
        _DATA_STORE[2]++;   // Add 1 hour
    } else {
        _time_carry(_DATA_STORE + 1);
    }

    if (_DATA_STORE[2] == 0x24) {   // Check hour
        _DATA_STORE[2] = 0x00;
        _DATA_STORE[3]++;   // Add 1 day
        _DATA_STORE[4]++;   // Add 1 date
    } else {
        _time_carry(_DATA_STORE + 2);
    }

    if (_DATA_STORE[3] == 0x08)     // Check day
        _DATA_STORE[3] = 0x01;

    switch (_DATA_STORE[4]) {       // Check date
    case 0x29:
        if (_DATA_STORE[5] == 0x02 && !_is_leap_year) {   // It's February
            _DATA_STORE[4] = 0x01;
            _DATA_STORE[5]++;
        }
        break;
    case 0x30:
        if (_DATA_STORE[5] == 0x02) {   // It's February
            _DATA_STORE[4] = 0x01;
            _DATA_STORE[5]++;
        }
        break;
    case 0x31:
        if (_DATA_STORE[5] == 0x04
                || _DATA_STORE[5] == 0x06
                || _DATA_STORE[5] == 0x09
                || _DATA_STORE[5] == 0x11) {    // Apr, Jun, Sep, Nov with 30 days
            _DATA_STORE[4] = 0x01;
            _DATA_STORE[5]++;
        }
        break;
    case 0x32:
        if (_DATA_STORE[5] == 0x01
                || _DATA_STORE[5] == 0x03
                || _DATA_STORE[5] == 0x05
                || _DATA_STORE[5] == 0x07
                || _DATA_STORE[5] == 0x08
                || _DATA_STORE[5] == 0x10
                || _DATA_STORE[5] == 0x12) {    // Jan, Mar, May, Jul, Aug, Oct, Dec with 31 days
            _DATA_STORE[4] = 0x01;
            _DATA_STORE[5]++;
        }
        break;
    default:
        _time_carry(_DATA_STORE + 4);
    }

    if (_DATA_STORE[5] == 0x13) {   // Check month
        _DATA_STORE[5] = 0x01;
        _DATA_STORE[6]++;   // Add 1 year
        _RTC_action_bits |= BIT2;   // Let's check the leap year later
    } else {
        _time_carry(_DATA_STORE + 5);
    }

    if (_DATA_STORE[6] == 0x9A) {   // Check year
        _DATA_STORE[6] = 0x00;
        _DATA_STORE[7]++;   // Add 1 century
    } else {
        _time_carry(_DATA_STORE + 6);
    }
    // After changes with the year
    if (_RTC_action_bits & BIT2) {
        // let's check the leap year property
        _check_leap_year();
        _RTC_action_bits &= ~BIT2;
    }

    if (_DATA_STORE[7] == 0x9A) {   // Check century
        _DATA_STORE[7] = 0x00;  // Century start over
    } else {
        _time_carry(_DATA_STORE + 7);
    }
}

/**
 * Deal with carry
 */
void _time_carry(unsigned char * byte) {
    _RTC_byte_l = *byte << 4;
    if (_RTC_byte_l == 0xA0) {
        _RTC_byte_h = *byte >> 4;
        _RTC_byte_h++;
        *byte = _RTC_byte_h << 4;
    }
}

/**
 * Alarm logic here
 */
void _check_alarms() {
    unsigned char day_mask_bit = 0x00;

    switch (_DATA_STORE[3]) {
    case 0x01:
        day_mask_bit = MON;
        break;
    case 0x02:
        day_mask_bit = TUE;
        break;
    case 0x03:
        day_mask_bit = WED;
        break;
    case 0x04:
        day_mask_bit = THU;
        break;
    case 0x05:
        day_mask_bit = FRI;
        break;
    case 0x06:
        day_mask_bit = SAT;
        break;
    case 0x07:
        day_mask_bit = SUN;
        break;
    }

    // Alarm 1
    if (_DATA_STORE[1] == _DATA_STORE[8] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[9] &&
            (_DATA_STORE[10] & 0x80 ||
                    _DATA_STORE[10] & day_mask_bit))
        _DATA_STORE[30] |= BIT0;
    // Alarm 2
    if (_DATA_STORE[1] == _DATA_STORE[11] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[12] &&
            (_DATA_STORE[13] & 0x80 ||
                    _DATA_STORE[13] & day_mask_bit))
        _DATA_STORE[30] |= BIT1;
    // Alarm 3
    if (_DATA_STORE[1] == _DATA_STORE[14] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[15] &&
            (_DATA_STORE[16] & 0x80 ||
                    _DATA_STORE[16] & day_mask_bit))
        _DATA_STORE[30] |= BIT2;
    // Alarm 4
    if (_DATA_STORE[1] == _DATA_STORE[17] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[18] &&
            (_DATA_STORE[19] & 0x80 ||
                    _DATA_STORE[19] & day_mask_bit))
        _DATA_STORE[30] |= BIT3;
    // Alarm 5
    if (_DATA_STORE[1] == _DATA_STORE[20] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[21] &&
            (_DATA_STORE[22] & 0x80 ||
                    _DATA_STORE[22] & day_mask_bit))
        _DATA_STORE[30] |= BIT4;
    // Alarm 6
    if (_DATA_STORE[1] == _DATA_STORE[23] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[24] &&
            (_DATA_STORE[25] & 0x80 ||
                    _DATA_STORE[25] & day_mask_bit))
        _DATA_STORE[30] |= BIT5;
}

/**
 * Check alarm interrupt flag and output interrupt
 */
void _alarm_interrupt() {
    unsigned char INT_uni = 0;
    unsigned char INT_A1 = 0, INT_A2 = 0, INT_A3 = 0;

    // Alarm 1
    if (_DATA_STORE[30] & BIT0 &&
            _DATA_STORE[29] & BIT0) {
        INT_uni = 1;
        if (_DATA_STORE[28] & 0x80)
            INT_A1 = 1;
    }
    // Alarm 2
    if (_DATA_STORE[30] & BIT1 &&
            _DATA_STORE[29] & BIT1) {
        INT_uni = 1;
        if (_DATA_STORE[28] & 0x80)
            INT_A2 = 1;
    }
    // Alarm 3
    if (_DATA_STORE[30] & BIT2 &&
            _DATA_STORE[29] & BIT2) {
        INT_uni = 1;
        if (_DATA_STORE[28] & 0x80)
            INT_A3 = 1;
    }
    // Alarm 4
    if (_DATA_STORE[30] & BIT3 &&
            _DATA_STORE[29] & BIT3 &&
            !INT_uni)
        INT_uni = 1;
    // Alarm 5
    if (_DATA_STORE[30] & BIT4 &&
            _DATA_STORE[29] & BIT4 &&
            !INT_uni)
        INT_uni = 1;
    // Alarm 6
    if (_DATA_STORE[30] & BIT5 &&
            _DATA_STORE[29] & BIT5 &&
            !INT_uni)
        INT_uni = 1;

    // Set the interrupt output pin to high
    if (INT_uni)
        P1OUT |= BIT5;
    if (INT_A1)
        P2OUT |= BIT0;
    if (INT_A2)
        P2OUT |= BIT1;
    if (INT_A3)
        P2OUT |= BIT2;
}

/**
 * Reset alarm interrupt output pin to low
 */
void _alarm_reset_interrupt() {
    P1OUT &= ~BIT5;
    P2OUT &= ~(BIT0 + BIT1 + BIT2);
}

/***********************************************
 * Mandatory functions for callback
 * You can modify codes in these functions
 *         to deal with data received
 *         or data to be sent
 *         but left function name unchanged
 ***********************************************/
unsigned char * USI_I2C_slave_TX_callback() {
    unsigned char _I2C_data_offset_1;
    _I2C_data_offset_1 = _I2C_data_offset;
    if (_I2C_data_offset_1 == 26)           // User reading the high part of the temperature result
        _TEMP_data_user_read = 1;
    if (_I2C_data_offset_1 == 27 &&
            _TEMP_data_user_read) {         // User reading the low part of the temperature result
        _DATA_STORE[28] &= ~BIT5;           // Clear temperature data ready bit
        _TEMP_data_user_read = 0;
    }
    _I2C_data_offset++;
    return _DATA_STORE + _I2C_data_offset_1;
}

unsigned char USI_I2C_slave_RX_callback(unsigned char * byte) {
    unsigned char byte_data;
    byte_data = *byte;
    if (!_USI_I2C_slave_n_byte) {
        _I2C_data_offset = byte_data;
        _USI_I2C_slave_n_byte = 1;
    } else {
        if (_I2C_data_offset != 26 &&
                _I2C_data_offset != 27) {
            switch(_I2C_data_offset) {
            case 28:    // Do not allow 1 on BIT5 when BIT5 in _DATA_STORE[28] is 0
                if (!(_DATA_STORE[28] & BIT5) &&
                        (byte_data & BIT5))
                    _DATA_STORE[28] = byte_data & ~BIT5;
                break;
            case 30:    // Do not allow 1 for alarm interrupt flags when flags are 0
                if (!(_DATA_STORE[30] & BIT0) &&
                        (byte_data & BIT0))
                    byte_data &= ~BIT0;
                if (!(_DATA_STORE[30] & BIT1) &&
                        (byte_data & BIT1))
                    byte_data &= ~BIT1;
                if (!(_DATA_STORE[30] & BIT2) &&
                        (byte_data & BIT2))
                    byte_data &= ~BIT2;
                if (!(_DATA_STORE[30] & BIT3) &&
                        (byte_data & BIT3))
                    byte_data &= ~BIT3;
                if (!(_DATA_STORE[30] & BIT4) &&
                        (byte_data & BIT4))
                    byte_data &= ~BIT4;
                if (!(_DATA_STORE[30] & BIT5) &&
                        (byte_data & BIT5))
                    byte_data &= ~BIT5;
                _DATA_STORE[30] = byte_data;
                break;
            default:
                *(_DATA_STORE + _I2C_data_offset) = byte_data;
            }
        }
        _I2C_data_offset++;
    }
    return 0;   // 0: No error; Not 0: Error in received data
}

void _USI_I2C_slave_reset_byte_count() {
    _USI_I2C_slave_n_byte = 0;
}
//**********************************************/

#ifdef _UART_OUTPUT
/**
 * Extra functions for software UART
 */
void _UART_TX_byte(unsigned char byte) {
    // Code bases on TI's example
    _UART_n_bit = 0xA;                  // Load Bit counter, 8data + ST/SP
    while (TACCR1 != TAR)               // Prevent async capture
        TACCR1 = TAR;                   // Current state of TA counter
    TACCR1 += _UART_period_1200;        // Some time till first bit
    _UART_TX_data = byte | 0x100;       // Add mark stop bit to _UART_TX_data
    _UART_TX_data = _UART_TX_data << 1; // Add space start bit
    TACCTL1 = (OUTMOD0 + CCIE);         // TXD = mark = idle
    while (TACCTL1 & CCIE);             // Wait for TX completion
}

void _UART_send_datetime() {
    unsigned int idx = 8;
    unsigned char byte_h, byte_l;
    while(idx) {
        byte_h = _DATA_STORE[idx - 1] >> 4;
        byte_l = _DATA_STORE[idx - 1] << 4;
        byte_l = byte_l >> 4;
        _UART_TX_byte(byte_h + 0x30);   // The higher 4 bits of 1 byte and converted to numeric ASCII
        _UART_TX_byte(byte_l + 0x30);   // The lower 4 bits of 1 byte and converted to numeric ASCII
        idx--;
    }
    _UART_TX_byte(0x0D);                // Extra carrige return
    _UART_TX_byte(0x0A);                // Display a new line after each datetime line
}
#endif

/**
 * The timer capture interrupt is set to happen every 0.5s
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void) {
    TACCR0 += _second_div;

    _second_tick++; // Increment the ticker
    switch (_second_tick) {
    case 2:
        _RTC_action_bits |= BIT4;   // Let's check alarm interrupt flag and output interrupt
        break;
    case 4:
#ifdef _UART_OUTPUT
        _RTC_action_bits |= BIT1;   // Let's send out data to UART
#endif
        break;
    case 6:
        _RTC_action_bits |= BIT5;   // Reset alarm interrupt output after 4 division period
        break;
    case 8:
        // Toggle P1.0 output level every 0.5s
        // to form a full 1-Hz square wave output
        P1OUT ^= BIT0;
        break;
    case 10:
        _RTC_action_bits2 |= BIT0;  // Send temperature ready interrupt if applicable
        break;
    case 12:
        _RTC_action_bits |= BIT0;   // Let's do time increment now
        break;
    case 14:
        _RTC_action_bits2 |= BIT1;  // Reset temperature ready interrupt
        break;
    case 16:
        // Toggle P1.0 output level every 0.5s
        // to form a full 1-Hz square wave output
        P1OUT ^= BIT0;
        _second_tick = 0;   // Reset ticker
    }
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
            TACCTL1 &= ~CCIE;           // All bits TXed, disable interrupt
        else {
            TACCTL1 |= OUTMOD2;         // TX Space
            if (_UART_TX_data & 0x01)
                TACCTL1 &= ~OUTMOD2;    // TX Mark
            _UART_TX_data = _UART_TX_data >> 1;
            _UART_n_bit--;
        }
    }
}
#endif

// ADC10 interrupt service routine for temperature convert
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    _RTC_action_bits |= BIT6;   // Temperature convert finished. Go on transfer data.
}
