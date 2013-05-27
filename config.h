/*
 * Global configuration for RTC and Temperature source
 *
 *  Created on: 2013-5-27
 *      Author: hyli
 */

#ifndef CONFIG_H_
#define CONFIG_H_

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
