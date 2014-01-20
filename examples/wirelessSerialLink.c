/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2014 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Wireless serial link.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "nRF905/nRF905.h"
#include "util.h"

#define PACKET_TYPE_DATA	0
#define PACKET_TYPE_ACK		1

#define MAX_PACKET_SIZE (NRF905_MAX_PAYLOAD - 2)
typedef struct {
	uint8_t dstAddress[NRF905_ADDR_SIZE];
	uint8_t type;
	uint8_t len;
	uint8_t data[MAX_PACKET_SIZE];
} packet_s;

static void sendPacket(packet_s*);
static bool getPacket(packet_s*);

void serialLink(void)
{
	util_init();

	// Start up
	nRF905_init();

	// Interrupts on
	sei();

	// Put into receive mode
	nRF905_receive();

	while(1)
	{
		packet_s packet;
		
		// Send serial data
		uint8_t dataSize;
		while((dataSize = uart_avail()))
		{
			// Make sure we don't try to send more than max packet size
			if(dataSize > MAX_PACKET_SIZE)
				dataSize = MAX_PACKET_SIZE;

			packet.type = PACKET_TYPE_DATA;
			packet.len = dataSize;

			// Copy data from serial to packet buffer
			for(uint8_t i=0;i<dataSize;i++)
			{
				uint8_t tmp;
				if(uart_get_nb(&tmp))
					packet.data[i] = tmp;
			}			

			// Send packet
			sendPacket(&packet);

			// Receive mode
			nRF905_receive();

			// Wait for ACK packet
			uint8_t startTime = millis();
			while(1)
			{
				bool timeout = false;
				while(1)
				{
					if(getPacket(&packet)) // Get new packet
						break;
					else if((uint8_t)(millis() - startTime) > 50) // 50ms timeout
					{
						timeout = true;
						break;
					}
				}

				if(timeout) // Timed out
				{
					puts("TO");
					break;
				}
				else if(packet.type == PACKET_TYPE_ACK) // Is packet type ACK?
					break;
			}
		}

		// Put into receive mode
		nRF905_receive();

		// Wait for data
		while(1)
		{
			if(getPacket(&packet) && packet.type == PACKET_TYPE_DATA) // Got a packet and is it a data packet?
			{
				// Print data
				for(uint8_t i=0;i<packet.len;i++)
					uart_put_nb(packet.data[i]);
					//printf("%c", packet.data[i]);

				// Reply with ACK
				packet.type = PACKET_TYPE_ACK;
				packet.len = 0;
				sendPacket(&packet);

				// Put into receive mode
				nRF905_receive();
			}
			else if(uart_avail()) // We've got some serial data, need to send it
				break;
		}
	}
}

// Send a packet
static void sendPacket(packet_s* packet)
{
	// Convert packet data to plain byte array
	uint8_t totalLength = packet->len + 2;
	uint8_t tmpBuff[totalLength];
	tmpBuff[0] = packet->type;
	tmpBuff[1] = packet->len;
	memcpy(&tmpBuff[2], packet->data, packet->len);

	// Set address of device to send to
	//nRF905_setTXAddress(packet->dstAddress);

	// Set payload data
	nRF905_setData(tmpBuff, totalLength);

	// Send payload (send fails if other transmissions are going on, keep trying until success)
	while(!nRF905_send());
}

// Get a packet
static bool getPacket(packet_s* packet)
{
	uint8_t buffer[NRF905_MAX_PAYLOAD];

	// See if any data available
	if(!nRF905_getData(buffer, sizeof(buffer)))
		return false;

	// Convert byte array to packet
	packet->type = buffer[0];
	packet->len = buffer[1];

	// Sanity check
	if(packet->len > MAX_PACKET_SIZE)
		packet->len = MAX_PACKET_SIZE;

	memcpy(packet->data, &buffer[2], packet->len);
	
	return true;
}