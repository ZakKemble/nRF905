/*
 * Project: nRF905 AVR/Arduino Library/Driver (Ping client example)
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Time how long it takes to send some data and get a reply
 * Should be around 14-17ms with default settings.
 * If the ping time is 0 or 1 then there is likely a problem with the client side wiring.
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

#define RXADDR 0xFE4CA6E5 // Address of this device
#define TXADDR 0x586F2E10 // Address of device to send to

#define TIMEOUT 1000 // 1 second ping timeout

#define PACKET_NONE		0
#define PACKET_OK		1
#define PACKET_INVALID	2

static volatile uint8_t packetStatus;

void NRF905_CB_RXCOMPLETE(void)
{
	packetStatus = PACKET_OK;
}

void NRF905_CB_RXINVALID(void)
{
	packetStatus = PACKET_INVALID;
}

void setup()
{
	Serial.begin(115200);
	Serial.println(F("Client started"));
	
	pinMode(A5, OUTPUT); // LED

	// Start up
	nRF905_init();
	
	// Set address of this device
	nRF905_setListenAddress(RXADDR);
}

void loop()
{
	static uint8_t counter;
	static uint32_t sent;
	static uint32_t replies;
	static uint32_t timeouts;
	static uint32_t invalids;

	// Make data
	char data[NRF905_MAX_PAYLOAD] = {0};
	sprintf(data, "test %hhu", counter);
	counter++;
	
	packetStatus = PACKET_NONE;

	Serial.print(F("Sending data: "));
	Serial.println(data);
	
	uint32_t startTime = millis();

	// Send the data (send fails if other transmissions are going on, keep trying until success) and enter RX mode on completion
	while(!nRF905_TX(TXADDR, data, sizeof(data), NRF905_NEXTMODE_RX));
	sent++;

	Serial.println(F("Data sent, waiting for reply..."));

	uint8_t success;

	// Wait for reply with timeout
	uint32_t sendStartTime = millis();
	while(1)
	{
		success = packetStatus;
		if(success != PACKET_NONE)
			break;
		else if(millis() - sendStartTime > TIMEOUT)
			break;
	}

	if(success == PACKET_NONE)
	{
		Serial.println(F("Ping timed out"));
		timeouts++;
	}
	else if(success == PACKET_INVALID)
	{
		Serial.println(F("Invalid packet!"));
		invalids++;
	}
	else
	{
		// If success toggle LED and send ping time over UART
		uint16_t totalTime = millis() - startTime;

		static uint8_t ledState;
		digitalWrite(A5, ledState ? HIGH : LOW);
		ledState = !ledState;

		replies++;

		Serial.print(F("Ping time: "));
		Serial.print(totalTime);
		Serial.println(F("ms"));

		// Get the ping data
		uint8_t replyData[NRF905_MAX_PAYLOAD];
		nRF905_read(replyData, sizeof(replyData));

		// Print out ping contents
		Serial.print(F("Data from server: "));
		Serial.write(replyData, sizeof(replyData));
		Serial.println();
	}

	Serial.print(F("Totals: "));
	Serial.print(sent);
	Serial.print(F(" Sent, "));
	Serial.print(replies);
	Serial.print(F(" Replies, "));
	Serial.print(timeouts);
	Serial.print(F(" Timeouts, "));
	Serial.print(invalids);
	Serial.println(F(" Invalid"));
	Serial.println(F("------"));

	delay(1000);
}
