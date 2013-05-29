/*
 * Global configuration for RTC and Temperature source
 *
 * HouYu Li <karajan_ii@hotmail.com>
 *
 * No license applied. Use as you wish.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/**
 * Uncomment following line to enable UART output
 * Comment for release build
 */
//#define _UART_OUTPUT
#ifdef _UART_OUTPUT

#define _UART_TXD	BIT2		// P1.2 as UART TXD

#endif

/**
 * Own I2C slave address
 */
#define _I2C_addr		0x41

/**
 * Day mask bit for alarm setting
 */
#define MON		BIT0
#define TUE		BIT1
#define WED		BIT2
#define THU		BIT3
#define FRI		BIT4
#define SAT		BIT5
#define SUN		BIT6

#endif /* CONFIG_H_ */
