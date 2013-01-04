/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Turn the nRF905 on to transmit some data, wait for a reply,
 * turn off and wait for a second.
 * Output power is set to the lowest setting, receive sensitivity is
 * lowered and uses the power up/down feature of the nRF905.
 */

#define F_CPU 8000000

#include "../nRF905/nRF905.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

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
#define TIMEOUT 1000 // 1 second ping timeout

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

	// Lowest transmit level -10db
	nRF905_setTransmitPower(NRF905_PWR_n10);
	
	// Reduce receive sensitivity to save a few mA
	nRF905_setLowRxPower(NRF905_LOW_RX_ENABLE);

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
		// Turn on module
		nRF905_powerUp();

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

		// If success toggle LED
		if(success)
		{
			PORTC ^= (1<<PORTC5);
			char str[] = "Got data\r\n";
			sendStr((byte*)str, sizeof(str));
		}
		else
		{
			char str[] = "Timeout\r\n";
			sendStr((byte*)str, sizeof(str));
		}	

		// Turn off module
		nRF905_powerDown();

		_delay_ms(1000);
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
