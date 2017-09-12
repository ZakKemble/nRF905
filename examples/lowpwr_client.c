/*
 * Project: nRF905 AVR/Arduino Library/Driver (Low power ping client example)
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Low power ping client
 *
 * Similar to the ping client example.
 * Power up the nRF905 to transmit some data, wait for a reply, turn off and wait for a second.
 * Output power is set to the lowest setting, receive sensitivity is lowered and uses the power up/down feature of the nRF905.
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

#if F_CPU > 16320000
	#define CLOCKSEL (_BV(CS02))
	#define TICKS ((F_CPU / 256) / 1000)
#elif F_CPU >= 64000
	#define CLOCKSEL (_BV(CS01)|_BV(CS00))
	#define TICKS ((F_CPU / 64) / 1000)
#else
	#error "F_CPU too low (< 64000), add more clock selects"
#endif

#define RXADDR 0xFE4CA6E5 // Address of this device
#define TXADDR 0x586F2E10 // Address of device to send to

#define TIMEOUT 1000 // 1 second ping timeout

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
static volatile uint32_t milliseconds;

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

uint32_t millis(void)
{
	uint32_t ms;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		ms = milliseconds;
	}
	return ms;
}

void main(void)
{
	clock_prescale_set(clock_div_1);

	// Timer 0 settings for approx. millisecond tracking
	TCCR0A = _BV(WGM01);
	TCCR0B = CLOCKSEL;
	TIMSK0 = _BV(OCIE0A);
	OCR0A = TICKS;

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

	uint8_t counter = 0;
	uint32_t sent = 0;
	uint32_t replies = 0;
	uint32_t timeouts = 0;
	uint32_t invalids = 0;

	while(1)
	{
		// Make data
		char data[NRF905_MAX_PAYLOAD] = {0};
		sprintf_P(data, PSTR("test %hhu"), counter);
		counter++;
		
		packetStatus = PACKET_NONE;
		
		printf_P(PSTR("Sending data: %s\n"), data);

		uint32_t startTime = millis();

		// Send the data (send fails if other transmissions are going on, keep trying until success) and enter RX mode on completion
		// If the radio is powered down then this function will take an additional 3ms to complete
		while(!nRF905_TX(TXADDR, data, sizeof(data), NRF905_NEXTMODE_RX));
		sent++;

		puts_P(PSTR("Data sent, waiting for reply..."));

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
			puts_P(PSTR("Ping timed out"));
			timeouts++;
		}
		else if(success == PACKET_INVALID)
		{
			// Got a corrupted packet
			puts_P(PSTR("Invalid packet!"));
			invalids++;
		}
		else
		{
			// If success toggle LED and send ping time over UART
			uint16_t totalTime = millis() - startTime;
			PORTC ^= _BV(PORTC5);

			replies++;

			printf_P(PSTR("Ping time: %ums\n"), totalTime);

			// Get the ping data
			uint8_t replyData[NRF905_MAX_PAYLOAD];
			nRF905_read(replyData, sizeof(replyData));

			// Print out ping contents
			printf_P(PSTR("Data from server: "));
			for(uint8_t i=0;i<sizeof(replyData);i++)
				printf_P(PSTR("%c"), replyData[i]);
			puts_P(PSTR(""));
		}

		// Turn off module
		nRF905_powerDown();

		printf_P(PSTR("Totals: %lu Sent, %lu Replies, %lu Timeouts, %lu Invalid\n------\n"), sent, replies, timeouts, invalids);

		_delay_ms(1000);			
	}
}

ISR(TIMER0_COMPA_vect)
{
	++milliseconds;
}
