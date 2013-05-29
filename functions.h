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

#ifdef _UART_OUTPUT
void _UART_TX_byte(unsigned char byte);
void _UART_send_datetime();
#endif

#endif /* FUNCTIONS_H_ */
