/*
 * Project: nRF905 AVR/Arduino Library/Driver (Debug example)
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Read configuration registers
 */

#define BAUD 115200

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include "nRF905.h"
#include "nRF905_defs.h"

#define RXADDR 0xFE4CA6E5 // Address of this device

static int put(char c, FILE* stream)
{
	if(c == '\n')
		put('\r', stream);
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;
}

static FILE uart_io = FDEV_SETUP_STREAM(put, NULL, _FDEV_SETUP_WRITE);

void main(void)
{
	// UART
	//PORTD |= _BV(PORTD0);
	//DDRD |= _BV(DDD1);
	UBRR0 = UBRR_VALUE;
#if USE_2X
	UCSR0A = _BV(U2X0);
#endif
	UCSR0B = _BV(TXEN0);

	stdout = &uart_io;

	// Start up
	nRF905_init();

	// Set address of this device
	nRF905_setListenAddress(RXADDR);

	// Interrupts on
	sei();

	// Put into receive mode
	nRF905_RX();
	
	while(1)
	{
		uint8_t regs[NRF905_REGISTER_COUNT];
		nRF905_getConfigRegisters(regs);
		printf_P(PSTR("Raw: "));
		
		uint8_t dataValid = 0;
		
		for(uint8_t i=0;i<NRF905_REGISTER_COUNT;i++)
		{
			printf_P(PSTR("%hhu "), regs[i]);
			if(regs[i] == 0xFF || regs[i] == 0x00)
				dataValid++;
		}

		puts_P(PSTR(""));

		// Registers were all 0xFF or 0x00,  this is probably bad
		if(dataValid >= NRF905_REGISTER_COUNT)
		{
			puts_P(PSTR("All registers read as 0xFF or 0x00! Is the nRF905 connected correctly?"));
			_delay_ms(1000);
			continue;
		}

		char* str;
		uint8_t data;

		uint16_t channel = ((uint16_t)(regs[1] & 0x01)<<8) | regs[0];
		uint32_t freq = (422400UL + (channel * 100UL)) * (1 + ((regs[1] & ~NRF905_MASK_BAND) >> 1));

		printf_P(PSTR("Channel: %u\n"), channel);
		printf_P(PSTR("Freq: %luKHz\n"), freq);
		printf_P(PSTR("Auto retransmit: %hhu\n"), !!(regs[1] & ~NRF905_MASK_AUTO_RETRAN));
		printf_P(PSTR("Low power RX: %hhu\n"), !!(regs[1] & ~NRF905_MASK_LOW_RX));

		// TX power
		data = regs[1] & ~NRF905_MASK_PWR;
		switch(data)
		{
			case NRF905_PWR_n10:
				data = -10;
				break;
			case NRF905_PWR_n2:
				data = -2;
				break;
			case NRF905_PWR_6:
				data = 6;
				break;
			case NRF905_PWR_10:
				data = 10;
				break;
			default:
				data = -127;
				break;
		}
		printf_P(PSTR("TX Power: %hhddBm\n"), (int8_t)data);
		
		// Freq band
		data = regs[1] & ~NRF905_MASK_BAND;
		switch(data)
		{
			case NRF905_BAND_433:
				str = (char*)"433";
				break;
			default:
				str = (char*)"868/915";
				break;
		}
		printf_P(PSTR("Band: %sMHz\n"), str);

		printf_P(PSTR("TX Address width: %hhu\n"), regs[2] >> 4);
		printf_P(PSTR("RX Address width: %hhu\n"), regs[2] & 0x07);

		printf_P(PSTR("RX Payload size: %hhu\n"), regs[3]);
		printf_P(PSTR("TX Payload size: %hhu\n"), regs[4]);

		printf_P(PSTR("RX Address [0]: %hhu\n"), regs[5]);
		printf_P(PSTR("RX Address [1]: %hhu\n"), regs[6]);
		printf_P(PSTR("RX Address [2]: %hhu\n"), regs[7]);
		printf_P(PSTR("RX Address [3]: %hhu\n"), regs[8]);
		printf_P(PSTR("RX Address: %lu\n"), ((unsigned long)regs[8]<<24 | (unsigned long)regs[7]<<16 | (unsigned long)regs[6]<<8 | (unsigned long)regs[5]));

		// CRC mode
		data = regs[9] & ~NRF905_MASK_CRC;
		switch(data)
		{
			case NRF905_CRC_16:
				str = (char*)"16bit";
				break;
			case NRF905_CRC_8:
				str = (char*)"8bit";
				break;
			default:
				str = (char*)"Disabled";
				break;
		}
		printf_P(PSTR("CRC Mode: %s\n"), str);

		// Xtal freq
		data = regs[9] & ~NRF905_MASK_CLK;
		switch(data)
		{
			case NRF905_CLK_4MHZ:
				data = 4;
				break;
			case NRF905_CLK_8MHZ:
				data = 8;
				break;
			case NRF905_CLK_12MHZ:
				data = 12;
				break;
			case NRF905_CLK_16MHZ:
				data = 16;
				break;
			case NRF905_CLK_20MHZ:
				data = 20;
				break;
			default:
				data = 0;
				break;
		}
		printf_P(PSTR("Xtal freq: %hhuMHz\n"), data);

		// Clock out freq
		data = regs[9] & ~NRF905_MASK_OUTCLK;
		switch(data)
		{
			case NRF905_OUTCLK_4MHZ:
				str = (char*)"4MHz";
				break;
			case NRF905_OUTCLK_2MHZ:
				str = (char*)"2MHz";
				break;
			case NRF905_OUTCLK_1MHZ:
				str = (char*)"1MHz";
				break;
			case NRF905_OUTCLK_500KHZ:
				str = (char*)"500KHz";
				break;
			default:
				str = "Disabled";
				break;
		}
		printf_P(PSTR("Clock out freq: %s\n"), str);
		
		puts_P(PSTR("---------------------"));

		_delay_ms(1000);				
	}
}
