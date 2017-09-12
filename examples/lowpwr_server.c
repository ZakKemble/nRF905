/*
 * Project: nRF905 AVR/Arduino Library/Driver (Low power ping server example)
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Low power ping server
 *
 * Similar to the ping server example
 * Output power is set to the lowest setting, receive sensitivity is lowered.
 */

#define BAUD 115200

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>
#include <util/setbaud.h>
#include <util/delay.h>
#include <stdio.h>
#include "nRF905.h"

#define RXADDR 0x586F2E10 // Address of this device
#define TXADDR 0xFE4CA6E5 // Address of device to send to

#define PACKET_NONE		0
#define PACKET_OK		1
#define PACKET_INVALID	2

static int put(char c, FILE* stream)
{
	if(c == '\n')
		put('\r', stream);
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;
}

static FILE uart_io = FDEV_SETUP_STREAM(put, NULL, _FDEV_SETUP_WRITE);

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

void main(void)
{
	clock_prescale_set(clock_div_1);

	// UART
	//PORTD |= _BV(PORTD0);
	//DDRD |= _BV(DDD1);
	UBRR0 = UBRR_VALUE;
#if USE_2X
	UCSR0A = _BV(U2X0);
#endif
	UCSR0B = _BV(TXEN0);

	// LED indicator
	DDRC |= _BV(DDC5);
	PORTC |= _BV(PORTC5);
	
	stdout = &uart_io;

	// Start up
	nRF905_init();

	// Set address of this device
	nRF905_setListenAddress(RXADDR);

	// Lowest transmit level -10db
	nRF905_setTransmitPower(NRF905_PWR_n10);

	// Reduce receive sensitivity to save a few mA
	nRF905_setLowRxPower(NRF905_LOW_RX_ENABLE);

	// Interrupts on
	sei();

	// Put into receive mode
	nRF905_RX();

	uint32_t pings = 0;
	uint32_t invalids = 0;

	while(1)
	{
		puts_P(PSTR("Waiting for ping..."));

		// Wait for data
		while(packetStatus == PACKET_NONE);
		
		if(packetStatus != PACKET_OK)
		{
			// Got a corrupted packet
			invalids++;
			packetStatus = PACKET_NONE;
			puts_P(PSTR("Invalid packet!\n"));
			nRF905_RX();
		}
		else
		{
			pings++;
			packetStatus = PACKET_NONE;
			
			// Make buffer for data
			uint8_t buffer[NRF905_MAX_PAYLOAD];
			nRF905_read(buffer, sizeof(buffer));

			puts_P(PSTR("Got ping, sending reply..."));

			// Send payload (send fails if other transmissions are going on, keep trying until success) and enter RX once complete
			while(!nRF905_TX(TXADDR, buffer, sizeof(buffer), NRF905_NEXTMODE_RX));

			puts_P(PSTR("Reply sent"));

			// Toggle LED
			PORTC ^= _BV(PORTC5);

			// Print out ping contents
			printf_P(PSTR("Data from client: "));
			for(uint8_t i=0;i<sizeof(buffer);i++)
				printf_P(PSTR("%c"), buffer[i]);
			puts_P(PSTR(""));
		}
		
		printf_P(PSTR("Totals: %lu Pings, %lu Invalid\n------\n"), pings, invalids);	
	}
}
