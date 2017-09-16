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

#define RXADDR 0xE7E7E7E7
#define TXADDR 0xE7E7E7E7

static int put(char c, FILE* stream)
{
	if(c == '\n')
		put('\r', stream);
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;
}

static FILE uart_io = FDEV_SETUP_STREAM(put, NULL, _FDEV_SETUP_WRITE);

// The NRF905_CB_RXCOMPLETE callback is ran once the packet has been received and is valid
void NRF905_CB_RXCOMPLETE(void)
{
	// Clear payload
	uint8_t buffer[NRF905_MAX_PAYLOAD];
	nRF905_read(buffer, sizeof(buffer));

	puts_P(PSTR("Got packet"));
}

// If NRF905_INTERRUPTS_AM is 1 in nRF905_config.h then the NRF905_CB_RXINVALID callback is ran once the packet has been received, but was corrupted (CRC failed)
void NRF905_CB_RXINVALID(void)
{
	puts_P(PSTR("Packet CRC failed"));
}

// If NRF905_INTERRUPTS_AM is 1 in nRF905_config.h then the NRF905_CB_ADDRMATCH callback is ran when the beginning of a new packet is detected (after a valid preamble and matching address)
void NRF905_CB_ADDRMATCH(void)
{
	puts_P(PSTR("Incoming packet"));
}

// The NRF905_CB_TXCOMPLETE callback is ran when a packet has finished transmitting
// This callback only works if the next mode is set to NRF905_NEXTMODE_STANDBY when calling nRF905_TX()
void NRF905_CB_TXCOMPLETE(void)
{
	nRF905_RX();
	puts_P(PSTR("Packet sent"));
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

	// Interrupts on
	sei();

	nRF905_RX();

	uint8_t testData[] = {
		2,
		3,
		4,
		5,
		6,
		7
	};

	while(1)
	{
		// Transmit some data every 500ms
		_delay_ms(500);

		nRF905_TX(TXADDR, testData, sizeof(testData), NRF905_NEXTMODE_STANDBY); // Transmit and go to standby mode once sent (the NRF905_CB_TXCOMPLETE callback only works if the next mode is standby)
		testData[0]++;

		// We're about to print some stuff to serial, however the callbacks also print to serial. The callbacks are ran from an interrupt which could run at any time (like in the middle of printing out the serial message below).
		// To make sure the callbacks don't run we temporarily turn the radio interrupt off.
		// When communicating with other SPI devices on the same bus as the radio then you should also wrap those sections in an NRF905_NO_INTERRUPT() block, this will stop the nRF905 interrupt from running and trying to use the bus at the same time.

		NRF905_NO_INTERRUPT()
		{
			// Print the message
			puts_P(PSTR("Packet send begin"));
		}
		// Radio interrupt is now back on, any callbacks waiting will now run
	}
}
