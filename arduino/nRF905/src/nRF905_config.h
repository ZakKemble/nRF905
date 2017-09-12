/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#ifndef NRF905_CONFIG_H_
#define NRF905_CONFIG_H_

// Crystal frequency (the one the radio module is using)
// NRF905_CLK_4MHZ
// NRF905_CLK_8MHZ
// NRF905_CLK_12MHZ
// NRF905_CLK_16MHZ
// NRF905_CLK_20MHZ
#define NRF905_CLK_FREQ		NRF905_CLK_16MHZ

// Use pin interrupt for data ready (DR)
// NOTE: If you have other devices connected that use the SPI bus then you will need to wrap those bits of code in NRF905_NO_INTERRUPT() blocks
#define NRF905_INTERRUPTS	1

// If you want to use the NRF905_CB_ADDRMATCH and NRF905_CB_RXINVALID callbacks with interrupts then both NRF905_INTERRUPTS and NRF905_INTERRUPTS_AM need to be enabled
#define NRF905_INTERRUPTS_AM	1

// If other libraries communicate with SPI devices while inside an interrupt then set this to 1, otherwise you can set this to 0
// If you're not sure then leave this at 1
// If this is 1 then global interrupts will be turned off when this library uses the SPI bus
#define NRF905_INT_SPI_COMMS	1

// Use software to get data ready state instead of reading pin for high/low state, this means you don't need to connect to DR pin
// This option can only be used if NRF905_INTERRUPTS is 0
#define NRF905_DR_SW		0

// Use software to get address match state instead of reading pin for high/low state, this means you don't need to connect to AM pin
// This option can only be used if NRF905_INTERRUPTS_AM is 0
#define NRF905_AM_SW		0

// Don't transmit if airway is busy (other transmissions are going on)
// This feature uses the CD pin
#define NRF905_COLLISION_AVOID	1


///////////////////
// Default radio settings
///////////////////

// Frequency
// Channel 0 is 422.4MHz for the 433MHz band, each channel increments the frequency by 100KHz, so channel 10 would be 423.4MHz
// Channel 0 is 844.8MHz for the 868/915MHz band, each channel increments the frequency by 200KHz, so channel 10 would be 846.8MHz
// Max channel is 511 (473.5MHz / 947.0MHz)
#define NRF905_CHANNEL			10

// Frequency band
// 868 and 915 are actually the same thing
// NRF905_BAND_433
// NRF905_BAND_868
// NRF905_BAND_915
#define NRF905_BAND			NRF905_BAND_433

// Output power
// n means negative, n10 = -10
// NRF905_PWR_n10 (-10dBm = 100uW)
// NRF905_PWR_n2 (-2dBm = 631uW)
// NRF905_PWR_6 (6dBm = 4mW)
// NRF905_PWR_10 (10dBm = 10mW)
#define NRF905_PWR			NRF905_PWR_10

// Save a few mA by reducing receive sensitivity
// NRF905_LOW_RX_DISABLE (Normal sensitivity)
// NRF905_LOW_RX_ENABLE (Lower sensitivity)
#define NRF905_LOW_RX		NRF905_LOW_RX_DISABLE

// Constantly retransmit payload while in transmit mode
// Can be useful in areas with lots of interference, but you'll need to make sure you can differentiate between re-transmitted packets and new packets (like an ID number).
// It will also block other transmissions if collision avoidance is enabled.
// NRF905_AUTO_RETRAN_DISABLE
// NRF905_AUTO_RETRAN_ENABLE
#define NRF905_AUTO_RETRAN	NRF905_AUTO_RETRAN_DISABLE

// Output a clock signal on pin 3 of IC
// NRF905_OUTCLK_DISABLE
// NRF905_OUTCLK_500KHZ
// NRF905_OUTCLK_1MHZ
// NRF905_OUTCLK_2MHZ
// NRF905_OUTCLK_4MHZ
#define NRF905_OUTCLK		NRF905_OUTCLK_DISABLE

// CRC checksum
// NRF905_CRC_DISABLE
// NRF905_CRC_8
// NRF905_CRC_16
#define NRF905_CRC			NRF905_CRC_16

// Address size
// The address is actually the SYNC part of the packet, just after the preamble and before the data
// NRF905_ADDR_SIZE_1 (not recommended, a lot of false invalid packets will be received)
// NRF905_ADDR_SIZE_4
#define NRF905_ADDR_SIZE	NRF905_ADDR_SIZE_4

// Payload size (1 - 32)
#define NRF905_PAYLOAD_SIZE	32 //NRF905_MAX_PAYLOAD


///////////////////
// Pin stuff
///////////////////

// Arduino pin assignments
#define NRF905_TRX_EN	7	// Enable/standby pin (Required)
#define NRF905_PWR_MODE	8	// Power mode pin (Required)
#define NRF905_TX_EN	9	// TX / RX mode pin (Required)
#define NRF905_CD		4	// Carrier detect pin (Optional, used for collision avoidance if NRF905_COLLISION_AVOID is 1 or if you want to use the nRF905_airwayBusy() function)
#define NRF905_CSN		10	// SPI slave select pin (Required)

// Data ready pin
// If using interrupts (NRF905_INTERRUPTS 1) then this must be an interrupt pin
// If NRF905_INTERRUPTS is 0 and NRF905_DR_SW is 1 then this pin does not need to be connected
#define NRF905_DR			3

// Address match pin
// If using AM interrupt (NRF905_INTERRUPTS_AM 1) then this must be an interrupt pin
// If NRF905_INTERRUPT_AM is 0 and NRF905_AM_SW is 1 then this pin does not need to be connected
#define NRF905_AM			2






// --------------------------------------
// Everything below here is for non-Arduino stuff
// --------------------------------------

// Enable/standby pin
#define NRF905_TRX_EN_PORT		D
#define NRF905_TRX_EN_BIT		7

// Power mode pin
#define NRF905_PWR_MODE_PORT	B
#define NRF905_PWR_MODE_BIT		0

// TX / RX mode pin
#define NRF905_TX_EN_PORT		B
#define NRF905_TX_EN_BIT		1

// Carrier detect pin (Optional, used for collision avoidance if NRF905_COLLISION_AVOID is 1 or if you want to use the nRF905_airwayBusy() function)
#define NRF905_CD_PORT			D
#define NRF905_CD_BIT			4

// Data ready pin
// If using interrupts (NRF905_INTERRUPTS 1) then this must be an external interrupt pin that matches the interrupt settings below.
// If NRF905_INTERRUPTS is 0 and NRF905_DR_SW is 1 then this pin does not need to be connected
#define NRF905_DR_PORT			D
#define NRF905_DR_BIT			3

// Address match pin
// If using AM interrupt (NRF905_INTERRUPTS_AM 1) then this must be an external interrupt pin that matches the interrupt settings below.
// If NRF905_INTERRUPT_AM is 0 and NRF905_AM_SW is 1 then this pin does not need to be connected
#define NRF905_AM_PORT			C
#define NRF905_AM_BIT			3

// SPI slave select pin
#define NRF905_CSN_PORT			D
#define NRF905_CSN_BIT			6


///////////////////
// **************************** NOT for Arduino ****************************
// Interrupt register stuff
// Only needed if NRF905_INTERRUPTS or NRF905_INTERRUPTS_AM are 1
///////////////////

// Interrupt number (INT0, INT1 etc)
// This must match the INT that the DR pin is connected to
#define NRF905_INTERRUPT_NUM_DR	1

// Interrupt number
// This must match the INT that the AM pin is connected to
#define NRF905_INTERRUPT_NUM_AM	0



// Leave these commented out to let the library figure out what registers to use

// Which interrupt to use for data ready (DR)
//#define NRF905_REG_EXTERNAL_INT_DR	EIMSK
//#define NRF905_BIT_EXTERNAL_INT_DR	INT1
//#define NRF905_INT_VECTOR_DR		INT1_vect

// Set interrupt to trigger on rising edge
//#define NRF905_REG_EXTERNAL_INT_CTL_DR	EICRA
//#define NRF905_BIT_EXTERNAL_INT_CTL_DR	(_BV(ISC11)|_BV(ISC10)) // Rising


// Which interrupt to use for address match (AM)
//#define NRF905_REG_EXTERNAL_INT_AM	EIMSK
//#define NRF905_BIT_EXTERNAL_INT_AM	INT0
//#define NRF905_INT_VECTOR_AM		INT0_vect

// Set interrupt to trigger on any change
//#define NRF905_REG_EXTERNAL_INT_CTL_AM	EICRA
//#define NRF905_BIT_EXTERNAL_INT_CTL_AM	(_BV(ISC00)) // Any change



#define NRF905_CONCAT(a, b) a ## b
#define NRF905_INTCONCAT(num) NRF905_CONCAT(INT, num)

#ifndef NRF905_REG_EXTERNAL_INT_DR
	#ifdef EIMSK
		#define NRF905_REG_EXTERNAL_INT_DR EIMSK
	#elif defined GICR
		#define NRF905_REG_EXTERNAL_INT_DR GICR
	#else
		#define NRF905_REG_EXTERNAL_INT_DR GIMSK
	#endif
#endif

#ifndef NRF905_BIT_EXTERNAL_INT_DR
	#define NRF905_BIT_EXTERNAL_INT_DR NRF905_INTCONCAT(NRF905_INTERRUPT_NUM_DR)
#endif

#ifndef NRF905_REG_EXTERNAL_INT_AM
	#ifdef EIMSK
		#define NRF905_REG_EXTERNAL_INT_AM EIMSK
	#elif defined GICR
		#define NRF905_REG_EXTERNAL_INT_AM GICR
	#else
		#define NRF905_REG_EXTERNAL_INT_AM GIMSK
	#endif
#endif

#ifndef NRF905_BIT_EXTERNAL_INT_AM
	#define NRF905_BIT_EXTERNAL_INT_AM NRF905_INTCONCAT(NRF905_INTERRUPT_NUM_AM)
#endif

#endif /* NRF905_CONFIG_H_ */
