/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 */

/*
 * Time how long it takes to send some data and get a reply
 * Should be around 14-16ms with default settings.
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

#define RXADDR 0xFE4CA6E5 // Address of this device (4 bytes / long data type)
#define TXADDR 0x586F2E10 // Address of device to send to (4 bytes / long data type)
#define TIMEOUT 1000 // 1 second ping timeout

// Just some random data to send
byte data[NRF905_MAX_PAYLOAD] = {
0x0A,
0x68,
0x45,
0xFA
};

void setup()
{
	// Start up
	nRF905_init();
	
	// Set address of this device
	nRF905_setRXAddress(RXADDR);

	// Put into receive mode
	nRF905_receive();

	Serial.begin(9600);
	
	Serial.println("Client started");
}

void loop()
{
	unsigned long startTime = millis();

	// Set address of device to send to
	nRF905_setTXAddress(TXADDR);

	// Set payload data
	nRF905_setData(data, sizeof(data));

	// Send payload (send fails if other transmissions are going on, keep trying until success)
	while(!nRF905_send());

	// Put into receive mode
	nRF905_receive();
	
	// Make buffer for reply
	byte buffer[NRF905_MAX_PAYLOAD];
	bool success;

	// Wait for reply with timeout
	unsigned long sendStartTime = millis();
	while(!(success = nRF905_getData(buffer, sizeof(buffer))) && millis() - sendStartTime < TIMEOUT);

	if(success)
	{
		unsigned int totalTime = millis() - startTime;
		Serial.print("Ping time: ");
		Serial.print(totalTime);
		Serial.println("ms");
	}
	else
		Serial.println("Ping timed out");
		
	delay(1000);
}