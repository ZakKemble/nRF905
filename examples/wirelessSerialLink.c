/*
 * Project: nRF905 AVR/Arduino Library/Driver (Wireless serial link example)
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Wireless serial link.
 *
 * TODO
 * Don't drop DATA/ACK packets
 * Buffer up incoming packets
 * Buffer up incoming UART data with a small timeout (~5ms?) before sending
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "nRF905.h"
#include "uart.h"

#if F_CPU > 16320000
	#define CLOCKSEL (_BV(CS02))
	#define TICKS ((F_CPU / 256) / 1000)
#elif F_CPU >= 64000
	#define CLOCKSEL (_BV(CS01)|_BV(CS00))
	#define TICKS ((F_CPU / 64) / 1000)
#else
	#error "F_CPU too low (< 64000), add more clock selects"
#endif

#define PACKET_TYPE_DATA	0
#define PACKET_TYPE_ACK		1

#define MAX_PACKET_SIZE (NRF905_MAX_PAYLOAD - 2)
typedef struct {
	uint8_t dstAddress[NRF905_ADDR_SIZE];
	uint8_t type;
	uint8_t len;
	uint8_t data[MAX_PACKET_SIZE];
} packet_t;

static volatile uint32_t milliseconds;

static volatile uint8_t newData[NRF905_MAX_PAYLOAD];
static volatile uint8_t gotNewData;

void NRF905_CB_RXCOMPLETE(void)
{
	gotNewData = 1;
	nRF905_read((uint8_t*)newData, sizeof(newData));
	
	// Still in RX mode
}

void NRF905_CB_TXCOMPLETE(void)
{
	nRF905_RX();
}

uint32_t millis(void)
{
	uint32_t ms;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		ms = milliseconds;
	}
	return ms;
}

// Send a packet
static void sendPacket(packet_t* packet)
{
	// Convert packet data to plain byte array
	uint8_t totalLength = packet->len + 2;
	uint8_t tmpBuff[totalLength];
	tmpBuff[0] = packet->type;
	tmpBuff[1] = packet->len;
	memcpy(&tmpBuff[2], packet->data, packet->len);

	// Set address of device to send to
	//nRF905_setTXAddress(packet->dstAddress);

	// Send payload (send fails if other transmissions are going on, keep trying until success)
	while(!nRF905_TX(NRF905_DEFAULT_TXADDR, tmpBuff, totalLength, NRF905_NEXTMODE_STANDBY));
}

// Get a packet
static uint8_t getPacket(packet_t* packet)
{
	// See if any data available
	if(!gotNewData)
		return 0;
	
	NRF905_NO_INTERRUPT()
	{
		gotNewData = 0;

		// Convert byte array to packet
		packet->type = newData[0];
		packet->len = newData[1];

		// Sanity check
		if(packet->len > MAX_PACKET_SIZE)
			packet->len = MAX_PACKET_SIZE;

		memcpy(packet->data, (uint8_t*)&newData[2], packet->len);
	}
	
	return 1;
}

void main(void)
{
	clock_prescale_set(clock_div_1);

	// Timer 0 settings for approx. millisecond tracking
	TCCR0A = _BV(WGM01);
	TCCR0B = CLOCKSEL;
	TIMSK0 = _BV(OCIE0A);
	OCR0A = TICKS;

	uart_init();

	// Start up
	nRF905_init();

	// Interrupts on
	sei();

	// Put into receive mode
	nRF905_RX();

	while(1)
	{
		packet_t packet;
		
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

			// Wait for ACK packet
			uint8_t startTime = millis();
			while(1)
			{
				uint8_t timedout = 0;
				while(1)
				{
					if(getPacket(&packet)) // Get new packet
						break;
					else if((uint8_t)(millis() - startTime) > 50) // 50ms timeout
					{
						timedout = 1;
						break;
					}
				}

				if(timedout) // Timed out
				{
					puts("TO");
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
				for(uint8_t i=0;i<packet.len;i++)
					uart_put_nb(packet.data[i]);
					//printf("%c", packet.data[i]);

				// Reply with ACK
				packet.type = PACKET_TYPE_ACK;
				packet.len = 0;
				sendPacket(&packet);
			}
			else if(uart_avail()) // We've got some serial data, need to send it
				break;
				
			// drop ACK type packets
		}
	}
}

ISR(TIMER0_COMPA_vect)
{
	++milliseconds;
}
