/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#include <avr/io.h>
#include "nRF905_spi.h"

void spi_init()
{
// NOTE: The SPI SS pin must be configured as an output for the SPI controller to run in master mode, even if you're using a different pin for SS!
// Also remember that some AVRs use different registers to enable pull-ups
// Don't forget to check nRF905_config.h to setup the rest of the pins

// SS = Output high
// MOSI = Output low
// MISO = Input with pullup
// SCK = Output low

#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || \
 defined(__AVR_ATmega48P__) || defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328P__)
	DDRB |= _BV(DDB2)|_BV(DDB3)|_BV(DDB5); // SS, MOSI and SCK as outputs
	PORTB |= _BV(PORTB4)|_BV(PORTB2); // Pullup enable on MISO, output HIGH on SS
	PORTB &= ~(_BV(PORTB3)|_BV(PORTB5)); // Make sure MOSI and SCK are outputting LOW
#elif defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || \
 defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
	DDRB |= _BV(DDB0)|_BV(DDB2)|_BV(DDB1);
	PORTB |= _BV(PORTB3)|_BV(PORTB0);
	PORTB &= ~(_BV(PORTB2)|_BV(PORTB1));
#elif defined(__AVR_ATmega164__) || defined(__AVR_ATmega324__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284__) || \
 defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
	DDRB |= _BV(DDB4)|_BV(DDB5)|_BV(DDB7);
	PORTB |= _BV(PORTB6)|_BV(PORTB4);
	PORTB &= ~(_BV(PORTB5)|_BV(PORTB7));
#else
	#error "Please setup the correct SPI pins for your microcontroller in nRF905_spi.c!"
#endif

	// nRF905 max SPI clock is 10MHz
	SPCR = _BV(SPE)|_BV(MSTR);
	SPSR = _BV(SPI2X);
}
