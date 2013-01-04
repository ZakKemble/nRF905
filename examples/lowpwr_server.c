/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Wait for data and reply.
 * Output power is set to the lowest setting, receive sensitivity is
 * lowered.
 */

#define F_CPU 8000000

#include "../nRF905/nRF905.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define RXADDR 0x586F2E10 // Address of this device (4 bytes / long data type)
#define TXADDR 0xFE4CA6E5 // Address of device to send to (4 bytes / long data type)

int main()
{
	// Start up
	nRF905_init();

	// Set address of this device
	nRF905_setRXAddress(RXADDR);

	// Lowest transmit level -10db
	nRF905_setTransmitPower(NRF905_PWR_n10);

	// Reduce receive sensitivity to save a few mA
	nRF905_setLowRxPower(NRF905_LOW_RX_ENABLE);

	// Interrupts on
	sei();

	// LED indicator
	DDRC |= (1<<DDC5);
	PORTC |= (1<<PORTC5);

	// Set address of device to send to
	nRF905_setTXAddress(TXADDR);

	// Put into receive mode
	nRF905_receive();

    while(1)
    {
		// Make buffer for data
		byte buffer[NRF905_MAX_PAYLOAD];

		// Wait for data
		while(!nRF905_getData(buffer, sizeof(buffer)));

		// Set payload data (reply with data received)
		nRF905_setData(buffer, sizeof(buffer));

		// Send payload (send fails if other transmissions are going on, keep trying until success)
		while(!nRF905_send());

		// Put back into receive mode
		nRF905_receive();

		// Toggle LED
		PORTC ^= (1<<PORTC5);	
    }
}
