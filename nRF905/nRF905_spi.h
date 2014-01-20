/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#ifndef NRF905_SPI_H_
#define NRF905_SPI_H_

#include <avr/io.h>

void spi_init(void);

inline void spi_transfer_nr(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & _BV(SPIF)));
}

inline uint8_t spi_transfer(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & _BV(SPIF)));
	return SPDR;
}

#endif /* NRF905_SPI_H_ */