/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2014 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#define BAUD 19200

#include <avr/io.h>
#include <util/atomic.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <stdbool.h>
#include "util.h"

#if F_CPU > 16320000
	#define CLOCKSEL (_BV(CS02))
	#define TICKS ((F_CPU / 256) / 1000)
#elif F_CPU >= 64000
	#define CLOCKSEL (_BV(CS01)|_BV(CS00))
	#define TICKS ((F_CPU / 64) / 1000)
#else
	#error "F_CPU too low (< 64000), add more clock selects"
#endif

#define UART_BUFFSIZE 128

typedef struct{
	uint8_t data[UART_BUFFSIZE];
	uint8_t head;
	uint8_t tail;
} s_ringBuffer;

static int put(char, FILE*);
FILE uart_io = FDEV_SETUP_STREAM(put, NULL, _FDEV_SETUP_WRITE);

static volatile s_ringBuffer rxBuffer;
static volatile s_ringBuffer txBuffer;
static volatile unsigned long milliseconds;

void util_init()
{
	// Timer 0 settings for approx. millisecond tracking
	TCCR0A = _BV(WGM01);
	TCCR0B = CLOCKSEL;
	TIMSK0 = _BV(OCIE0A);
	OCR0A = TICKS;

	// UART
	PORTD |= _BV(PORTD0);

	UBRR0 = UBRR_VALUE;
#if USE_2X
	UCSR0A = _BV(U2X0);
#endif
	UCSR0B = _BV(TXEN0)|_BV(RXEN0)|_BV(RXCIE0)|_BV(UDRIE0);

	stdout = &uart_io;
}

uint8_t uart_avail()
{
	int8_t diff = rxBuffer.head - rxBuffer.tail;
	if(diff < 0)
		diff += UART_BUFFSIZE;
	return (uint8_t)diff;
}

// Get next value
bool uart_get_nb(uint8_t* b)
{
	UCSR0B &= ~_BV(RXCIE0);

	// Empty
	if(rxBuffer.head == rxBuffer.tail)
	{
		UCSR0B |= _BV(RXCIE0);
		return false;
	}	

	*b = rxBuffer.data[rxBuffer.tail];
	if (++rxBuffer.tail >= UART_BUFFSIZE)
		rxBuffer.tail = 0;
	
	UCSR0B |= _BV(RXCIE0);

	return true;
}

void uart_put_nb(uint8_t b)
{
	put((char)b, NULL);
}

static int put(char c, FILE* stream)
{
	(void)(stream); // Get rid of unused variable warning

	//if(UCSR0A & _BV(UDRE0))
	//	UDR0 = c;
	//else
	//{
		UCSR0B &= ~_BV(UDRIE0);
		int8_t diff = txBuffer.head - txBuffer.tail;
		if(diff < 0)
			diff += UART_BUFFSIZE;

		if(diff < UART_BUFFSIZE - 1)
		{
			txBuffer.data[txBuffer.head] = c;
			if (++txBuffer.head >= UART_BUFFSIZE)
				txBuffer.head = 0;
		}
	//}

	UCSR0B |= _BV(UDRIE0);
	UCSR0A |= _BV(TXC0);

	return 0;
}

unsigned long millis()
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

ISR(USART_RX_vect)
{
	int8_t diff = rxBuffer.head - rxBuffer.tail;
	if(diff < 0)
		diff += UART_BUFFSIZE;

	if(diff < UART_BUFFSIZE - 1)
	{
		rxBuffer.data[rxBuffer.head] = UDR0;
		if (++rxBuffer.head >= UART_BUFFSIZE)
			rxBuffer.head = 0;
	}
	else // Buffer full
		UDR0;
}

ISR(USART_UDRE_vect)
{
	if(txBuffer.head != txBuffer.tail)
	{
		UDR0 = txBuffer.data[txBuffer.tail];
		if (++txBuffer.tail >= UART_BUFFSIZE)
			txBuffer.tail = 0;
	}
	else // Empty
		UCSR0B &= ~_BV(UDRIE0);
}
