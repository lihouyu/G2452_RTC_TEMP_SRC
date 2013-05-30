/*
 * Function definitions
 *
 * HouYu Li <karajan_ii@hotmail.com>
 *
 * No license applied. Use as you wish.
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

void _init_DS();
void _check_leap_year();
void _time_increment();

/***********************************************
 * Mandatory functions for callback
 ***********************************************/
unsigned char * USI_I2C_slave_TX_callback();
unsigned char USI_I2C_slave_RX_callback(unsigned char * byte);
void _USI_I2C_slave_reset_byte_count();
//**********************************************/

#ifdef _UART_OUTPUT
void _UART_TX_byte(unsigned char byte);
void _UART_send_datetime();
#endif

#endif /* FUNCTIONS_H_ */
