/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2014 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#ifndef UTIL_H_
#define UTIL_H_

void util_init(void);
uint8_t uart_avail(void);
bool uart_get_nb(uint8_t*);
void uart_put_nb(uint8_t);
unsigned long millis(void);

#endif /* UTIL_H_ */