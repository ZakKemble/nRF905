/*
 * Project: nRF905 AVR/Arduino Library/Driver (UART stuff for wireless serial link example)
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#ifndef UART_H_
#define UART_H_

void uart_init(void);
uint8_t uart_avail(void);
uint8_t uart_get_nb(uint8_t*);
void uart_put_nb(uint8_t);

#endif /* UART_H_ */
