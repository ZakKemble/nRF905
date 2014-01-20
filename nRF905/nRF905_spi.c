/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#include <avr/io.h>
#include "nRF905_spi.h"

void spi_init()
{
	DDRB |= _BV(DDB2)|_BV(DDB3)|_BV(DDB5);
	PORTB |= _BV(PORTB4)|_BV(PORTB2);
	PORTB &= ~(_BV(PORTB3)|_BV(PORTB5));

	// nRF905 max SPI clock is 10MHz
	SPCR = _BV(SPE)|_BV(MSTR);
	SPSR = _BV(SPI2X);
}
