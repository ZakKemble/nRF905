/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 */

#ifndef SPI_H_
#define SPI_H_

#include "nRF905_types.h"
#include <avr/io.h>

void spi_init();

byte inline spi_transfer(byte data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

#endif /* SPI_H_ */