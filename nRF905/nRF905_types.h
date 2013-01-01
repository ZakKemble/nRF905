/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 */

#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#ifndef ARDUINO
#include <stdbool.h>
typedef unsigned char byte;
#endif

typedef struct
{
//	union{
		byte reg1;
/*		byte unused:2;
		bool auto_retran:1;
		bool low_pwr:1;
		byte pa_pwr:2;
		bool pll:1;
		byte channel:1;
	};*/
//	union{
		byte reg2;
/*		bool crc_mode:1;
		bool crc_en:1;
		byte crystal:3;
		bool clk_out_en:1;
		byte clk_out_freq:2;
	};*/
	byte payloadSize;
}s_nrf905_config;

#endif /* TYPEDEFS_H_ */
