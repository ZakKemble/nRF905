/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 */

/*
 * Wait for data and reply.
 *
 * 7 -> CE
 * 8 -> PWR
 * 9 -> TXE
 * 2 -> CD
 * 3 -> DR
 * 10 -> CSN
 * 12 -> SO
 * 11 -> SI
 * 13 -> SCK
 */

#include <nRF905.h>
#include <SPI.h>

#define RXADDR 0x586F2E10 // Address of this device (4 bytes / long data type)
#define TXADDR 0xFE4CA6E5 // Address of device to send to (4 bytes / long data type)

void setup()
{
	// Start up
	nRF905_init();
	
	// Set address of this device
	nRF905_setRXAddress(RXADDR);

	// Put into receive mode
	nRF905_receive();

	Serial.begin(9600);

	Serial.println("Server started");
}

void loop()
{
	Serial.println("Waiting for ping...");

	// Make buffer for data
	byte buffer[NRF905_MAX_PAYLOAD];

	// Wait for data
	while(!nRF905_getData(buffer, sizeof(buffer)));

	// Set address of device to send to
	nRF905_setTXAddress(TXADDR);

	// Set payload data (reply with data received)
	nRF905_setData(buffer, sizeof(buffer));

	// Send payload (send fails if other transmissions are going on, keep trying until success)
	while(!nRF905_send());

	// Put back into receive mode
	nRF905_receive();

	Serial.println("Got ping");
}