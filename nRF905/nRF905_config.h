/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 */

#ifndef NRF905_CONFIG_H_
#define NRF905_CONFIG_H_

// Crystal frequency (the one the radio IC/module is using) (NRF905_CLK_4MHZ / NRF905_CLK_8MHZ / NRF905_CLK_12MHZ / NRF905_CLK_16MHZ / NRF905_CLK_20MHZ)
#define NRF905_CLK_FREQ		NRF905_CLK_16MHZ

// Use pin interrupt for data ready
#define NRF905_INTERRUPTS	1

// Use software to get address match state instead of high/low connection
#define NRF905_AM_SW		0

// Use software to get data ready state instead of high/low connection
// Interrupts can not be used with software data ready
#define NRF905_DR_SW		0

// Don't transmit if airway is busy
#define NRF905_COLLISION_AVOID	1

#ifdef ARDUINO

// Arduino pins
#define TRX_EN		7
#define PWR_MODE	8
#define TX_EN		9
#define CD			2
#define AM			4
#define DR			3
#define CSN			10

#else
// Non-Arduino pins

// Enable/standby pin
#define CFG_TRX_EN_PORT		D
#define CFG_TRX_EN_BIT		7

// Power mode pin
#define CFG_PWR_MODE_PORT	B
#define CFG_PWR_MODE_BIT	0

// TX / RX mode pin
#define CFG_TX_EN_PORT		B
#define CFG_TX_EN_BIT		1

// Carrier detect pin
#define CFG_CD_PORT			D
#define CFG_CD_BIT			2

// Address match pin
#define CFG_AM_PORT			D
#define CFG_AM_BIT			4

// Data ready pin
#define CFG_DR_PORT			D
#define CFG_DR_BIT			3

// SPI slave select pin
#define CFG_CSN_PORT		B
#define CFG_CSN_BIT			2

#endif

// Interrupt register stuff

// Which interrupt to use for data ready (DR)
#define REG_EXTERNAL_INT	EIMSK
#define BIT_EXTERNAL_INT	INT1
#define INT_VECTOR			INT1_vect

// Set interrupt to trigger on rising edge
#define REG_EXTERNAL_INT_CTL	EICRA
#define BIT_EXTERNAL_INT_CTL	((1<<ISC11)|(1<<ISC10))

/*
// Frequency
#define NRF905_FREQ			435000000UL

// Frequency band (NRF905_BAND_433MHZ / NRF905_BAND_868MHZ / NRF905_BAND_915MHZ)
#define NRF905_BAND			NRF905_BAND_433MHZ

// Output power (NRF905_PWR_n10 / NRF905_PWR_n2 / NRF905_PWR_6 / NRF905_PWR_10) n means negative, n10 = -10
#define NRF905_PWR			NRF905_PWR_10

// Save a few mA by reducing receive sensitivity (NRF905_LOW_RX_DISABLE / NRF905_LOW_RX_ENABLE)
#define NRF905_LOW_RX		NRF905_LOW_RX_DISABLE

// Constantly retransmit payload while in transmit mode (NRF905_AUTO_RETRAN_DISABLE / NRF905_AUTO_RETRAN_ENABLE)
#define NRF905_AUTO_RETRAN	NRF905_AUTO_RETRAN_DISABLE

// Output a clock signal on pin 3 of IC (NRF905_OUTCLK_DISABLE / NRF905_OUTCLK_ENABLE)
#define NRF905_OUTCLK		NRF905_OUTCLK_DISABLE

// Clock signal frequency (NRF905_OUTCLK_500KHZ / NRF905_OUTCLK_1MHZ / NRF905_OUTCLK_2MHZ / NRF905_OUTCLK_4MHZ)
#define NRF905_OUTCLK_FREQ	NRF905_OUTCLK_500KHZ

// CRC checksum (NRF905_CRC_DISABLE / NRF905_CRC_ENABLE)
#define NRF905_CRC			NRF905_CRC_ENABLE

// CRC checksum mode (NRF905_CRC_MODE_8 / NRF905_CRC_MODE_16)
#define NRF905_CRC_MODE		NRF905_CRC_MODE_16

// Payload size (1 - 32)
#define NRF905_PAYLOAD_SIZE	32
*/

#endif /* NRF905_CONFIG_H_ */