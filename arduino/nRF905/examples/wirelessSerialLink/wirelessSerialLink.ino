/*
 * Project: nRF905 AVR/Arduino Library/Driver (Wireless serial link example)
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Wireless serial link
 *
 * TODO
 * Don't drop DATA/ACK packets
 * Buffer up incoming packets
 * Buffer up incoming UART data with a small timeout (~5ms?) before sending
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

#define PACKET_TYPE_DATA	0
#define PACKET_TYPE_ACK		1

#define MAX_PACKET_SIZE (NRF905_MAX_PAYLOAD - 2)
typedef struct {
	byte dstAddress[NRF905_ADDR_SIZE];
	byte type;
	byte len;
	byte data[MAX_PACKET_SIZE];
} packet_t;

static volatile byte newData[NRF905_MAX_PAYLOAD];
static volatile bool gotNewData;

void NRF905_CB_RXCOMPLETE(void)
{
	gotNewData = true;
	nRF905_read((byte*)newData, sizeof(newData));
	
	// Still in RX mode
}

void NRF905_CB_TXCOMPLETE(void)
{
	nRF905_RX();
}

void setup()
{
	// Start up
	nRF905_init();

	// Put into receive mode
	nRF905_RX();

	Serial.begin(19200);
	Serial.println(F("Ready"));
}

void loop()
{
	packet_t packet;

	// Send serial data
	byte dataSize;
	while((dataSize = Serial.available()))
	{
		// Make sure we don't try to send more than max packet size
		if(dataSize > MAX_PACKET_SIZE)
			dataSize = MAX_PACKET_SIZE;

		packet.type = PACKET_TYPE_DATA;
		packet.len = dataSize;

		// Copy data from serial to packet buffer
		for(byte i=0;i<dataSize;i++)
			packet.data[i] = Serial.read();

		// Send packet
		sendPacket(&packet);

		// Wait for ACK packet
		byte startTime = millis();
		while(1)
		{
			bool timeout = false;
			while(1)
			{
				if(getPacket(&packet)) // Get new packet
					break;
				else if((byte)(millis() - startTime) > 50) // 50ms timeout
				{
					timeout = true;
					break;
				}
			}

			if(timeout) // Timed out
			{
				Serial.println(F("TO"));
				break;
			}
			else if(packet.type == PACKET_TYPE_ACK) // Is packet type ACK?
				break;
			
			// drop DATA type packets
		}
	}

	// Wait for data
	while(1)
	{
		if(getPacket(&packet) && packet.type == PACKET_TYPE_DATA) // Got a packet and is it a data packet?
		{
			// Print data
			Serial.write(packet.data, packet.len);

			// Reply with ACK
			packet.type = PACKET_TYPE_ACK;
			packet.len = 0;
			sendPacket(&packet);
		}
		else if(Serial.available()) // We've got some serial data, need to send it
			break;
		
		// drop ACK type packets
	}
}

// Send a packet
static void sendPacket(void* _packet)
{
	// Void pointer to packet_t pointer hack
	// Arduino puts all the function defs at the top of the file before packet_t being declared :/
	packet_t* packet = (packet_t*)_packet;

	// Convert packet data to plain byte array
	byte totalLength = packet->len + 2;
	byte tmpBuff[totalLength];
	tmpBuff[0] = packet->type;
	tmpBuff[1] = packet->len;
	memcpy(&tmpBuff[2], packet->data, packet->len);

	// Set address of device to send to
	//nRF905_setTXAddress(packet->dstAddress);

	// Send payload (send fails if other transmissions are going on, keep trying until success)
	while(!nRF905_TX(NRF905_DEFAULT_TXADDR, tmpBuff, totalLength, NRF905_NEXTMODE_STANDBY));
}

// Get a packet
static bool getPacket(void* _packet)
{
	// Void pointer to packet_t pointer hack
	// Arduino puts all the function defs at the top of the file before packet_t being declared :/
	packet_t* packet = (packet_t*)_packet;

	// See if any data available
	if(!gotNewData)
		return false;
	
	NRF905_NO_INTERRUPT()
	{
		gotNewData = false;

		// Convert byte array to packet
		packet->type = newData[0];
		packet->len = newData[1];

		// Sanity check
		if(packet->len > MAX_PACKET_SIZE)
			packet->len = MAX_PACKET_SIZE;

		memcpy(packet->data, (byte*)&newData[2], packet->len);
	}
	
	return true;
}