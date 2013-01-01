/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 */

#ifndef NRF905_H_
#define NRF905_H_

#ifdef ARDUINO
#include "Arduino.h"
#else
#include "nRF905_types.h"
#endif

// Setting options
#define NRF905_BAND_433			0x00
#define NRF905_BAND_868			0x02
#define NRF905_BAND_915			0x02
#define NRF905_PWR_n10			0x00
#define NRF905_PWR_n2			0x04
#define NRF905_PWR_6			0x08
#define NRF905_PWR_10			0x0C
#define NRF905_LOW_RX_ENABLE	0x10
#define NRF905_LOW_RX_DISABLE	0x00
#define NRF905_AUTO_RETRAN_ENABLE	0x20
#define NRF905_AUTO_RETRAN_DISABLE	0x00
#define NRF905_OUTCLK_ENABLE	0x04
#define NRF905_OUTCLK_DISABLE	0x00
#define NRF905_OUTCLK_4MHZ		0x00
#define NRF905_OUTCLK_2MHZ		0x01
#define NRF905_OUTCLK_1MHZ		0x02
#define NRF905_OUTCLK_500KHZ	0x03
#define NRF905_CRC_ENABLE		0x40
#define NRF905_CRC_DISABLE		0x00
#define NRF905_CRC_MODE_8		0x00
#define NRF905_CRC_MODE_16		0x80

#define NRF905_MAX_PAYLOAD		32

// Start up
void nRF905_init();

// Frequency / channel
// Channel can be 0 - 511
// 433MHz band: 422.4MHz - 473.5MHz, 100KHz steps
// 868/915MHz band: 844.8MHz - 947MHz, 200KHz steps

// Set frequency, workout the channel from the frequency
void nRF905_setFrequency(byte, unsigned long);

// Alternatively, just set the channel, this skips having to workout the channel from the frequency
void nRF905_setChannel(byte, unsigned int);

// Set auto retransmit
void nRF905_setAutoRetransmit(byte);

// Set low power receive
void nRF905_setLowRxPower(byte);

// Set output power
void nRF905_setTransmitPower(byte);

// 8/16bit CRC
void nRF905_setCRCMode(byte);

// Set CRC
void nRF905_setCRC(byte);

// Set clock output
void nRF905_setClockOut(byte);

// Clock output frequency, higher clock uses more power
void nRF905_setClockOutFreq(byte);

// Payload size
void nRF905_setPayloadSize(byte);

// Send to address
void nRF905_setTXAddress(long);

// Address of this device
void nRF905_setRXAddress(long);

// Set payload data
void nRF905_setData(byte*, byte);

// See if other transmissions are going on
bool nRF905_airwayBusy();

// Send the payload
// Returns false if other transmissions are going on and collision detection is enabled
bool nRF905_send();

// Block until transmission finished
void nRF905_waitForTransmitEnd();

// Receive mode
void nRF905_receive();

// Get received payload if available
bool nRF905_getData(byte*, byte);

// Turn on
void nRF905_powerUp();

// Turn off
void nRF905_powerDown();

// Enter standby
void nRF905_enterStandBy();

// Leave standby
void nRF905_leaveStandBy();

#endif /* NRF905_H_ */