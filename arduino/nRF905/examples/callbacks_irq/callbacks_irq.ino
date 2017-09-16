/*
 * Project: nRF905 AVR/Arduino Library/Driver (Callbacks example)
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Example showing all callbacks and turning IRQ off and on
 */

#include <nRF905.h>

#define RXADDR 0xE7E7E7E7
#define TXADDR 0xE7E7E7E7

// The NRF905_CB_RXCOMPLETE callback is ran once the packet has been received and is valid
void NRF905_CB_RXCOMPLETE(void)
{
	// Clear payload
	uint8_t buffer[NRF905_MAX_PAYLOAD];
	nRF905_read(buffer, sizeof(buffer));

	// Printing to serial inside an interrupt is bad!
	// If the serial buffer fills up the program will lock up!
	// Don't do this in your program, this only works here because we're not printing too much data
	Serial.println(F("Got packet"));
}

// If NRF905_INTERRUPTS_AM is 1 in nRF905_config.h then the NRF905_CB_RXINVALID callback is ran once the packet has been received, but was corrupted (CRC failed)
void NRF905_CB_RXINVALID(void)
{
	// Printing to serial inside an interrupt is bad!
	// If the serial buffer fills up the program will lock up!
	// Don't do this in your program, this only works here because we're not printing too much data
	Serial.println(F("Packet CRC failed"));
}

// If NRF905_INTERRUPTS_AM is 1 in nRF905_config.h then the NRF905_CB_ADDRMATCH callback is ran when the beginning of a new packet is detected (after a valid preamble and matching address)
void NRF905_CB_ADDRMATCH(void)
{
	// Printing to serial inside an interrupt is bad!
	// If the serial buffer fills up the program will lock up!
	// Don't do this in your program, this only works here because we're not printing too much data
	Serial.println(F("Incoming packet"));
}

// The NRF905_CB_TXCOMPLETE callback is ran when a packet has finished transmitting
// This callback only works if the next mode is set to NRF905_NEXTMODE_STANDBY when calling nRF905_TX()
void NRF905_CB_TXCOMPLETE(void)
{
	// Printing to serial inside an interrupt is bad!
	// If the serial buffer fills up the program will lock up!
	// Don't do this in your program, this only works here because we're not printing too much data
	Serial.println(F("Packet sent"));
}

void setup()
{
	Serial.begin(115200);

	// Start up
	nRF905_init();

	nRF905_RX();
}

void loop()
{
	static uint8_t testData[] = {
		2,
		3,
		4,
		5,
		6,
		7
	};

	// Transmit some data every 500ms
	delay(500);

	nRF905_TX(TXADDR, testData, sizeof(testData), NRF905_NEXTMODE_STANDBY); // Transmit and go to standby mode once sent (the NRF905_CB_TXCOMPLETE callback only works if the next mode is standby)
	testData[0]++;

	// We're about to print some stuff to serial, however the callbacks also print to serial. The callbacks are ran from an interrupt which could run at any time (like in the middle of printing out the serial message below).
	// To make sure the callbacks don't run we temporarily turn the radio interrupt off.
	// When communicating with other SPI devices on the same bus as the radio then you should also wrap those sections in an NRF905_NO_INTERRUPT() block, this will stop the nRF905 interrupt from running and trying to use the bus at the same time.

	NRF905_NO_INTERRUPT()
	{
		// Print the message
		Serial.println(F("Packet send begin"));
	}
	// Radio interrupt is now back on, any callbacks waiting will now run
}
