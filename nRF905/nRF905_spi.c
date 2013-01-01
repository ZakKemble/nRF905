/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 */

#include "nRF905_spi.h"
#include <avr/io.h>

void spi_init()
{
	DDRB |= (1<<DDB2)|(1<<DDB3)|(1<<DDB5);
	PORTB |= (1<<PORTB4)|(1<<PORTB2);
	PORTB &= ~((1<<PORTB3)|(1<<PORTB5));

	// nRF905 max SPI clock is 10MHz
	SPCR = (1<<SPE)|(1<<MSTR);
	SPSR = (1<<SPI2X);
}
