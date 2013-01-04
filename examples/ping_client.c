/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Time how long it takes to send some data and get a reply
 * Should be around 14-16ms with default settings
 */

#define F_CPU 8000000

#include "../nRF905/nRF905.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdlib.h>

#if F_CPU > 16320000
	#define CLOCKSEL (1<<CS02)
	#define TICKS ((F_CPU / 256) / 1000)
#elif F_CPU >= 64000
	#define CLOCKSEL (1<<CS01)|(1<<CS00)
	#define TICKS ((F_CPU / 64) / 1000)
#else
	#error "F_CPU too low (< 64000), add more clock selects"
#endif

#define BAUD 9600
#include <util/setbaud.h>

#define RXADDR 0xFE4CA6E5 // Address of this device (4 bytes / long data type)
#define TXADDR 0x586F2E10 // Address of device to send to (4 bytes / long data type)
#define TIMEOUT 100 // 1 second ping timeout

static void sendStr(byte*, byte);
static unsigned long millis();

// Just some random data to send
static byte data[NRF905_MAX_PAYLOAD] = {
	0x0A,
	0x68,
	0x45,
	0xFA
};

static volatile unsigned long milliseconds = 0;

int main()
{
	// Timer 0 settings for approx. millisecond tracking
	TCCR0A = (1<<WGM01);
	TCCR0B = CLOCKSEL;
	TIMSK0 = (1<<OCIE0A);
	OCR0A = TICKS;

	// UART
	PORTD &= ~(1<<PORTD0);
	UBRR0 = UBRR_VALUE;
#if USE_2X
	UCSR0A = (1<<U2X0);
#endif
	UCSR0B = (1<<TXEN0);
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);

	// Start up
	nRF905_init();

	// Set address of this device
	nRF905_setRXAddress(RXADDR);

	// Interrupts on
	sei();

	// LED indicator
	DDRC |= (1<<DDC5);
	PORTC |= (1<<PORTC5);

	// Set address of device to send to
	nRF905_setTXAddress(TXADDR);

	// Set payload data
	nRF905_setData(data, sizeof(data));

	// Put into receive mode
	nRF905_receive();

    while(1)
    {
		unsigned long startTime = millis();

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

		// If success toggle LED and send ping time over UART
		if(success)
		{
			unsigned int totalTime = millis() - startTime;
			PORTC ^= (1<<PORTC5);
			char str[] = "    ms\r\n";
			if(totalTime > 9999)
				totalTime = 9999;
			itoa(totalTime, str, 10);
			sendStr((byte*)str, sizeof(str));
		}
		else
		{
			char str[] = "Timeout\r\n";
			sendStr((byte*)str, sizeof(str));
		}

		_delay_ms(100);					
    }
}

static void sendStr(byte* str, byte len)
{
	for(byte i=0;i<len;i++)
	{
		UDR0 = str[i];
		while (!(UCSR0A & (1<<UDRE0)));
	}
}

static unsigned long millis()
{
	unsigned long ms;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		ms = milliseconds;
	}
	return ms;
}

ISR(TIMER0_COMPA_vect)
{
	++milliseconds;
}
