/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, me@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#ifdef ARDUINO
#include "Arduino.h"
#include "SPI.h"
#else
#include "nRF905_spi.h"
#endif
#include "nRF905.h"
#include "nRF905_defs.h"
#include "nRF905_types.h"
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#if NRF905_INTERRUPTS
#define NRF905_INT_ON() REG_EXTERNAL_INT |= (1<<BIT_EXTERNAL_INT)
#define NRF905_INT_OFF() REG_EXTERNAL_INT &= ~(1<<BIT_EXTERNAL_INT)
#else
#define NRF905_INT_ON()
#define NRF905_INT_OFF()
#endif

static void setConfigReg1(byte, byte, byte);
static void setConfigReg2(byte, byte, byte);
static void setConfigRegister(byte, byte);
//static void config();
static void setAddress(long, byte);
//static bool inline airwayBusy();
//static bool receiveBusy();
static bool dataReady();
#if NRF905_AM_SW || NRF905_DR_SW
static byte readStatus();
#endif
static void inline disableStandbyMode();
static void inline enableStandbyMode();
static void inline receiveMode();
static void inline transmitMode();
static void inline spiSelect();
static void inline spiDeselect();
#ifdef ARDUINO
static byte inline spi_transfer(byte);
#endif

static s_nrf905_config config;

#if NRF905_INTERRUPTS
static volatile byte rxBuffer[NRF905_MAX_PAYLOAD];
static volatile bool gotData;
#endif

void nRF905_init()
{
#ifdef ARDUINO
	pinMode(TRX_EN, OUTPUT);
	pinMode(PWR_MODE, OUTPUT);
	pinMode(TX_EN, OUTPUT);
	pinMode(CD, INPUT);
#if !NRF905_AM_SW
	pinMode(AM, INPUT);
#endif
#if !NRF905_DR_SW
	pinMode(DR, INPUT);
#endif

	pinMode(CSN, OUTPUT);
	digitalWrite(CSN, HIGH);

	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV2);
#else
	TRX_EN_DDR |= (1<<TRX_EN_BIT);
	PWR_MODE_DDR |= (1<<PWR_MODE_BIT);
	TX_EN_DDR |= (1<<TX_EN_BIT);

	CD_DDR &= ~(1<<CD_BIT);
#if !NRF905_AM_SW
	AM_DDR &= ~(1<<AM_BIT);
#endif
#if !NRF905_DR_SW
	DR_DDR &= ~(1<<DR_BIT);
#endif

	spi_init();
#endif

	// Set default values
	config.reg1 = 0x00;
	config.reg2 = (0xC7 | NRF905_CLK_FREQ);
	config.payloadSize = NRF905_MAX_PAYLOAD;

#if NRF905_INTERRUPTS
	memset((byte*)rxBuffer, 0, sizeof(rxBuffer));
	gotData = false;

	// Set interrupts
	REG_EXTERNAL_INT_CTL |= BIT_EXTERNAL_INT_CTL;
	NRF905_INT_ON();
#endif

	// Startup
	nRF905_powerUp();
	receiveMode();
	enableStandbyMode();

//	config();

	// 4 byte TX and RX addresses
	setConfigRegister(NRF905_CMD_W_CONFIG | NRF905_REG_ADDR_WIDTH, 0b01000100);

	// Default config
	nRF905_setFrequency(NRF905_BAND_433, 433200000);
	nRF905_setAutoRetransmit(NRF905_AUTO_RETRAN_DISABLE);
	nRF905_setLowRxPower(NRF905_LOW_RX_DISABLE);
	nRF905_setTransmitPower(NRF905_PWR_10);
	nRF905_setCRCMode(NRF905_CRC_MODE_16);
	nRF905_setCRC(NRF905_CRC_ENABLE);
	nRF905_setClockOut(NRF905_OUTCLK_DISABLE);
	nRF905_setClockOutFreq(NRF905_OUTCLK_500KHZ);
	nRF905_setPayloadSize(NRF905_MAX_PAYLOAD);
	nRF905_setTXAddress(0xE7E7E7E7);
	nRF905_setRXAddress(0xE7E7E7E7);
}

// Frequency / channel
// Channel can be 0 - 511
// 433MHz band: 422.4MHz - 473.5MHz, 100KHz steps
// 868/915MHz band: 844.8MHz - 947MHz, 200KHz steps

// Set frequency, workout the channel from the frequency
void nRF905_setFrequency(byte band, unsigned long freq)
{
	nRF905_setChannel(band, NRF905_CALC_CHANNEL(freq, band));
}

// Set channel
void nRF905_setChannel(byte band, unsigned int channel)
{
	config.reg1 = (config.reg1 & NRF905_MASK_CHANNEL) | band | ((channel & 0x100)>>8);

	enableStandbyMode();
	NRF905_INT_OFF();
	spiSelect();
	spi_transfer(NRF905_CMD_W_CONFIG | NRF905_REG_CHANNEL);
	spi_transfer(channel);
	spi_transfer(config.reg1);
	spiDeselect();
	NRF905_INT_ON();
}

// Set auto retransmit
void nRF905_setAutoRetransmit(byte val)
{
	setConfigReg1(val, NRF905_MASK_AUTO_RETRAN, NRF905_REG_AUTO_RETRAN);
}

// Set low power receive
void nRF905_setLowRxPower(byte val)
{
	setConfigReg1(val, NRF905_MASK_LOW_RX, NRF905_REG_LOW_RX);
}

// Set output power
void nRF905_setTransmitPower(byte val)
{
	setConfigReg1(val, NRF905_MASK_PWR, NRF905_REG_PWR);
}

// 8/16bit CRC
void nRF905_setCRCMode(byte val)
{
	setConfigReg2(val, NRF905_MASK_CRC_MODE, NRF905_REG_CRC_MODE);
}

// Set CRC
void nRF905_setCRC(byte val)
{
	setConfigReg2(val, NRF905_MASK_CRC, NRF905_REG_CRC);
}

// Set clock output
void nRF905_setClockOut(byte val)
{
	setConfigReg2(val, NRF905_MASK_OUTCLK, NRF905_REG_OUTCLK);
}

// Clock output frequency, higher clock uses more power
void nRF905_setClockOutFreq(byte val)
{
	setConfigReg2(val, NRF905_MASK_OUTCLK_FREQ, NRF905_REG_OUTCLK_FREQ);
}

// Payload size
void nRF905_setPayloadSize(byte size)
{
	NRF905_INT_OFF();
	if(size > NRF905_MAX_PAYLOAD)
		size = NRF905_MAX_PAYLOAD;
	config.payloadSize = size;

	enableStandbyMode();
	spiSelect();
	spi_transfer(NRF905_CMD_W_CONFIG | NRF905_REG_RX_PAYLOAD_SIZE);
	spi_transfer(size);
	spi_transfer(size);
	spiDeselect();
	NRF905_INT_ON();
}

static void setConfigReg1(byte val, byte mask, byte reg)
{
	config.reg1 = (config.reg1 & mask) | val;
	setConfigRegister(NRF905_CMD_W_CONFIG | reg, config.reg1);
}

static void setConfigReg2(byte val, byte mask, byte reg)
{
	config.reg2 = (config.reg2 & mask) | val;
	setConfigRegister(NRF905_CMD_W_CONFIG | reg, config.reg2);
}

static void setConfigRegister(byte cmd, byte val)
{
	enableStandbyMode();
	NRF905_INT_OFF();
	spiSelect();
	spi_transfer(cmd);
	spi_transfer(val);
	spiDeselect();
	NRF905_INT_ON();
}
/*
// Set configuration
static void config()
{
	spiSelect();
	spi_transfer(NRF905_CMD_W_CONFIG);
	spi_transfer((NRF905_CHAN>>1));
	spi_transfer(NRF905_AUTO_RETRAN | NRF905_LOW_RX | NRF905_PWR | NRF905_BAND | (NRF905_CHAN & 0x01));
	spi_transfer(0b01000100); // 4 byte TX and RX addresses
	spi_transfer(NRF905_PAYLOAD_SIZE); // RX payload size
	spi_transfer(NRF905_PAYLOAD_SIZE); // TX payload size
	spi_transfer(0xE7); // Default receive address
	spi_transfer(0xE7);
	spi_transfer(0xE7);
	spi_transfer(0xE7);
	spi_transfer(NRF905_CRC_MODE | NRF905_CRC | NRF905_CLK_FREQ | NRF905_OUTCLK | NRF905_OUTCLK_FREQ);
	spiDeselect();
}
*/
// Set address of device to send to
void nRF905_setTXAddress(long address)
{
	setAddress(address, NRF905_CMD_W_TX_ADDRESS);
}

// Set address for this device
void nRF905_setRXAddress(long address)
{
	setAddress(address, NRF905_CMD_W_CONFIG | NRF905_REG_RX_ADDRESS);
}

// Set address
static void setAddress(long address, byte cmd)
{
	enableStandbyMode();
	NRF905_INT_OFF();
	spiSelect();
	spi_transfer(cmd);
	spi_transfer(address>>24);
	spi_transfer(address>>16);
	spi_transfer(address>>8);
	spi_transfer(address);
	spiDeselect();
	NRF905_INT_ON();
}

// Set the payload data
void nRF905_setData(byte* data, byte len)
{
	if(len > config.payloadSize)
		len = config.payloadSize;

	// Put into stand by mode
	enableStandbyMode();

	NRF905_INT_OFF();
	spiSelect();
	spi_transfer(NRF905_CMD_W_TX_PAYLOAD);

	// Load data
	for(byte i=0;i<len;i++)
		spi_transfer(data[i]);

	spiDeselect();
	NRF905_INT_ON();
}

//#if NRF905_COLLISION_AVOID
// See if airway is busy (carrier detect pin high)
bool nRF905_airwayBusy()
{
#ifdef ARDUINO
	return digitalRead(CD);
#else
	return (CD_PORT & (1<<CD_BIT));
#endif
}
/*
// See if device is receiving something meant for it
// Hardware: Address match pin high
// Software: Address match status bit set
static bool receiveBusy()
{
#if NRF905_AM_SW
	return (readStatus() & (1<<NRF905_STATUS_AM));
#else
#ifdef ARDUINO
	return digitalRead(AM);
#else
	return (AM_PORT & (1<<AM_BIT));
#endif
#endif
}
*/
//#endif

// See if data ready, true if received new data/finished transmitting
// Hardware: Data ready pin high
// Software: Data ready status bit set
static bool dataReady()
{
#if NRF905_DR_SW
	return (readStatus() & (1<<NRF905_STATUS_DR));
#else
#ifdef ARDUINO
	return digitalRead(DR);
#else
	return (DR_PORT & (1<<DR_BIT));
#endif
#endif
}

// Transmit payload
bool nRF905_send()
{
#if NRF905_COLLISION_AVOID
	// Don't transmit if busy
	if(nRF905_airwayBusy())
		return false;
#endif

	// Put into transmit mode
	transmitMode();

	// Enable chip
	disableStandbyMode();
	
	// Need to give it time to get into transmit mode
	_delay_us(650);

	return true;
}

// Block until transmission has finished
void nRF905_waitForTransmitEnd()
{
	while(!dataReady());
}

// Put into receive mode
void nRF905_receive()
{
	receiveMode();
	disableStandbyMode();
}

// Get received data if available
bool nRF905_getData(byte* data, byte len)
{
	if(len > config.payloadSize)
		len = config.payloadSize;

#if NRF905_INTERRUPTS
	// No data received
	if(!gotData)
		return false;

	NRF905_INT_OFF();
	// Copy and clear data buffer
	memcpy(data, (byte*)rxBuffer, len);
	memset((byte*)rxBuffer, 0, sizeof(rxBuffer));
	gotData = false;
	NRF905_INT_ON();
#else
	// No data ready
	if(!dataReady())
		return false;

	spiSelect();
	spi_transfer(NRF905_CMD_R_RX_PAYLOAD);

	// Get received payload
	for(byte i=0;i<len;i++)
		data[i] = spi_transfer(NRF905_CMD_NOP);

	// Must make sure all of the payload has been read, otherwise DR never goes low
	byte remaining = config.payloadSize - len;
	for(byte i=0;i<remaining;i++)
		spi_transfer(NRF905_CMD_NOP);

	spiDeselect();
#endif

	return true;
}

// Power up
void nRF905_powerUp()
{
#ifdef ARDUINO
	digitalWrite(PWR_MODE, HIGH);
#else
	PWR_MODE_PORT |= (1<<PWR_MODE_BIT);
#endif
	
	// Give it time to turn on
	_delay_ms(3);
}

// Power down
void nRF905_powerDown()
{
#ifdef ARDUINO
	digitalWrite(PWR_MODE, LOW);
#else
	PWR_MODE_PORT &= ~(1<<PWR_MODE_BIT);
#endif
}

void nRF905_enterStandBy()
{
	enableStandbyMode();
}

void nRF905_leaveStandBy()
{
	disableStandbyMode();
}

#if NRF905_AM_SW || NRF905_DR_SW
// Read status register
static byte readStatus()
{
	byte status;
	NRF905_INT_OFF();
	spiSelect();
	status = spi_transfer(NRF905_CMD_NOP);
	spiDeselect();
	NRF905_INT_ON();
	return status;
}
#endif

static void inline disableStandbyMode()
{
#ifdef ARDUINO
	digitalWrite(TRX_EN, HIGH);
#else
	TRX_EN_PORT |= (1<<TRX_EN_BIT);
#endif
}

static void inline enableStandbyMode()
{
#ifdef ARDUINO
	digitalWrite(TRX_EN, LOW);
#else
	TRX_EN_PORT &= ~(1<<TRX_EN_BIT);
#endif
}

static void inline receiveMode()
{
#ifdef ARDUINO
	digitalWrite(TX_EN, LOW);
#else
	TX_EN_PORT &= ~(1<<TX_EN_BIT);
#endif
}

static void inline transmitMode()
{
#ifdef ARDUINO
	digitalWrite(TX_EN, HIGH);
#else
	TX_EN_PORT |= (1<<TX_EN_BIT);
#endif
}

static void inline spiSelect()
{
#ifdef ARDUINO
	digitalWrite(CSN, LOW);
#else
	CSN_PORT &= ~(1<<CSN_BIT);
#endif
}

static void inline spiDeselect()
{
#ifdef ARDUINO
	digitalWrite(CSN, HIGH);
#else
	CSN_PORT |= (1<<CSN_BIT);
#endif
}

#ifdef ARDUINO
static byte inline spi_transfer(byte data)
{
	return SPI.transfer(data);
}
#endif

#if NRF905_INTERRUPTS
// Data ready pin interrupt
ISR(INT_VECTOR, ISR_NOBLOCK)
{
	NRF905_INT_OFF();

	spiSelect();
	spi_transfer(NRF905_CMD_R_RX_PAYLOAD);

	// Get the received payload
	byte size = config.payloadSize;
	for(byte i=0;i<size;i++)
		rxBuffer[i] = spi_transfer(NRF905_CMD_NOP);

	spiDeselect();

	gotData = true;

	NRF905_INT_ON();
}
#endif
