/*
 * Project: nRF905 AVR/Arduino Library/Driver (Ping server example)
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Ping server
 *
 * Listen for packets and send them back
 *
 * 7 -> CE
 * 8 -> PWR
 * 9 -> TXE
 * 4 -> CD
 * 3 -> DR
 * 2 -> AM
 * 10 -> CSN
 * 12 -> SO
 * 11 -> SI
 * 13 -> SCK
 */

#include <nRF905.h>

#define RXADDR 0x586F2E10 // Address of this device
#define TXADDR 0xFE4CA6E5 // Address of device to send to

#define PACKET_NONE		0
#define PACKET_OK		1
#define PACKET_INVALID	2

static volatile uint8_t packetStatus;

void NRF905_CB_RXCOMPLETE(void)
{
	packetStatus = PACKET_OK;
	nRF905_standby();
}

void NRF905_CB_RXINVALID(void)
{
	packetStatus = PACKET_INVALID;
	nRF905_standby();
}

void setup()
{
	Serial.begin(115200);
	Serial.println(F("Server started"));

	pinMode(A5, OUTPUT); // LED

	// Start up
	nRF905_init();
	
	// Set address of this device
	nRF905_setListenAddress(RXADDR);

	// Put into receive mode
	nRF905_RX();
}

void loop()
{
	static uint32_t pings;
	static uint32_t invalids;

	Serial.println(F("Waiting for ping..."));

	// Wait for data
	while(packetStatus == PACKET_NONE);

	if(packetStatus != PACKET_OK)
	{
		invalids++;
		Serial.println(F("Invalid packet!"));
		packetStatus = PACKET_NONE;
		nRF905_RX();
	}
	else
	{
		pings++;
		packetStatus = PACKET_NONE;

		// Make buffer for data
		uint8_t buffer[NRF905_MAX_PAYLOAD];
		nRF905_read(buffer, sizeof(buffer));

		Serial.println(F("Got ping, sending reply..."));

		// Send back the data, once the transmission has completed go into receive mode
		while(!nRF905_TX(TXADDR, buffer, sizeof(buffer), NRF905_NEXTMODE_RX));

		Serial.println(F("Reply sent"));

		// Toggle LED
		static uint8_t ledState;
		digitalWrite(A5, ledState ? HIGH : LOW);
		ledState = !ledState;

		// Print out ping contents
		Serial.print(F("Data from server: "));
		Serial.write(buffer, sizeof(buffer));
		Serial.println();
	}

	Serial.print(F("Totals: "));
	Serial.print(pings);
	Serial.print(F(" Pings, "));
	Serial.print(invalids);
	Serial.println(F(" Invalid"));
	Serial.println(F("------"));
}
