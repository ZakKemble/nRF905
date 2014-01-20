/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Turn the nRF905 on to transmit some data, wait for a reply,
 * turn off and wait for a second.
 * Output power is set to the lowest setting, receive sensitivity is
 * lowered and uses the power up/down feature of the nRF905.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "nRF905/nRF905.h"
#include "util.h"

#define RXADDR {0xFE, 0x4C, 0xA6, 0xE5} // Address of this device (4 bytes)
#define TXADDR {0x58, 0x6F, 0x2E, 0x10} // Address of device to send to (4 bytes)
	
#define TIMEOUT 1000 // 1 second ping timeout

void lowpwr_client(void)
{
	util_init();

	// Start up
	nRF905_init();

	// Set address of this device
	uint8_t addrRx[] = RXADDR;
	nRF905_setRXAddress(addrRx);

	// Lowest transmit level -10db
	nRF905_setTransmitPower(NRF905_PWR_n10);
	
	// Reduce receive sensitivity to save a few mA
	nRF905_setLowRxPower(NRF905_LOW_RX_ENABLE);

	// Interrupts on
	sei();

	// LED indicator
	DDRC |= _BV(DDC5);
	PORTC |= _BV(PORTC5);

	// Put into receive mode
	nRF905_receive();
	
	uint8_t counter = 0;

	while(1)
	{
		// Make data
		char data[NRF905_MAX_PAYLOAD] = {0};
		sprintf_P(data, PSTR("test %hhu"), counter);
		counter++;

		// Turn on module
		nRF905_powerUp();

		unsigned long startTime = millis();

		// Set address of device to send to
		uint8_t addr[] = TXADDR;
		nRF905_setTXAddress(addr);

		// Set payload data
		nRF905_setData(data, sizeof(data));

		// Send payload (send fails if other transmissions are going on, keep trying until success)
		while(!nRF905_send());

		// Put into receive mode
		nRF905_receive();

		// Make buffer for reply
		uint8_t buffer[NRF905_MAX_PAYLOAD];
		bool success;

		// Wait for reply with timeout
		unsigned long sendStartTime = millis();
		while(1)
		{
			success = nRF905_getData(buffer, sizeof(buffer));
			if(success)// Got data
				break;

			// Timeout
			if(millis() - sendStartTime > TIMEOUT)
				break;
		}

		// Turn off module
		nRF905_powerDown();

		// If success toggle LED and send ping time over UART
		if(success)
		{
			unsigned int totalTime = millis() - startTime;
			PORTC ^= _BV(PORTC5);

			printf_P(PSTR("Ping time: %ums\n"), totalTime);

			// Print out ping contents
			printf_P(PSTR("Data from server: "));
			for(uint8_t i=0;i<sizeof(buffer);i++)
				uart_put_nb(buffer[i]);
			puts_P(PSTR(""));
		}
		else
			puts_P(PSTR("Ping timed out"));

		_delay_ms(100);					
	}
}
