/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Wait for data and reply.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "nRF905/nRF905.h"
#include "util.h"

#define RXADDR {0x58, 0x6F, 0x2E, 0x10} // Address of this device (4 bytes)
#define TXADDR {0xFE, 0x4C, 0xA6, 0xE5} // Address of device to send to (4 bytes)

void ping_server(void)
{
	util_init();

	// Start up
	nRF905_init();

	// Set address of this device
	uint8_t addrRx[] = RXADDR;
	nRF905_setRXAddress(addrRx);

	// Interrupts on
	sei();

	// LED indicator
	DDRC |= _BV(DDC5);
	PORTC |= _BV(PORTC5);

	// Put into receive mode
	nRF905_receive();

	while(1)
	{
		puts_P(PSTR("Waiting for ping..."));
		
		// Make buffer for data
		uint8_t buffer[NRF905_MAX_PAYLOAD];

		// Wait for data
		while(!nRF905_getData(buffer, sizeof(buffer)));
		
		puts_P(PSTR("Got ping"));

		// Set address of device to send to
		uint8_t addr[] = TXADDR;
		nRF905_setTXAddress(addr);

		// Set payload data (reply with data received)
		nRF905_setData(buffer, sizeof(buffer));
		
		puts_P(PSTR("Sending reply..."));

		// Send payload (send fails if other transmissions are going on, keep trying until success)
		while(!nRF905_send());

		// Put back into receive mode
		nRF905_receive();
		
		puts_P(PSTR("Reply sent"));

		// Toggle LED
		PORTC ^= _BV(PORTC5);

		// Print out ping contents
		printf_P(PSTR("Data: "));
		for(uint8_t i=0;i<sizeof(buffer);i++)
			uart_put_nb(buffer[i]);
		puts_P(PSTR(""));	
	}
}
