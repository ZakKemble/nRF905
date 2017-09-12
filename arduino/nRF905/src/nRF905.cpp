/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#else
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "nRF905_spi.h"
#endif

#include <string.h>
#include <stdint.h>
#include "nRF905.h"
#include "nRF905_config.h"
#include "nRF905_defs.h"

#define noinline __attribute__ ((__noinline__))

#define NEED_SW_STATUS_SUPPORT (NRF905_AM_SW || NRF905_DR_SW)

#define NRF905_USE_PWR		1 // TODO

#if !NRF905_USE_PWR
#define POWERED_UP()	(1)
#define POWER_UP()		(void)(0)
#define POWER_DOWN()	(void)(0)
#endif

#ifdef ARDUINO

#define delay_ms(ms) delay(ms)
#define delay_us(us) delayMicroseconds(us)

#if NRF905_USE_PWR
#define POWERED_UP()			(digitalRead(NRF905_PWR_MODE) == HIGH)
#define POWER_UP()				(digitalWrite(NRF905_PWR_MODE, HIGH))
#define POWER_DOWN()			(digitalWrite(NRF905_PWR_MODE, LOW))
#endif

#define STANDBY_LEAVE()			(digitalWrite(NRF905_TRX_EN, HIGH))
#define STANDBY_ENTER()			(digitalWrite(NRF905_TRX_EN, LOW))
#define MODE_RX()				(digitalWrite(NRF905_TX_EN, LOW))
#define MODE_TX()				(digitalWrite(NRF905_TX_EN, HIGH))
#define spiSelect()				(digitalWrite(NRF905_CSN, LOW))
#define spiDeselect()			(digitalWrite(NRF905_CSN, HIGH))
#define spi_transfer(data)		(SPI.transfer(data))
#define spi_transfer_nr(data)	(SPI.transfer(data))

#else
	
#define	delay_ms(ms) _delay_ms(ms)
#define delay_us(us) _delay_us(us)

#if NRF905_USE_PWR
#define POWERED_UP()	(PWR_MODE_PORT & _BV(PWR_MODE_BIT))
#define POWER_UP()		(PWR_MODE_PORT |= _BV(PWR_MODE_BIT))
#define POWER_DOWN()	(PWR_MODE_PORT &= ~_BV(PWR_MODE_BIT))
#endif

#define STANDBY_LEAVE()	(TRX_EN_PORT |= _BV(TRX_EN_BIT))
#define STANDBY_ENTER()	(TRX_EN_PORT &= ~_BV(TRX_EN_BIT))
#define MODE_RX()		(TX_EN_PORT &= ~_BV(TX_EN_BIT))
#define MODE_TX()		(TX_EN_PORT |= _BV(TX_EN_BIT))
#define spiSelect()		(CSN_PORT &= ~_BV(CSN_BIT))
#define spiDeselect()	(CSN_PORT |= _BV(CSN_BIT))

#endif

#if defined(__cplusplus)
extern "C" {
#endif
	static void __empty_callback0(void){}
#if defined(__cplusplus)
}
#endif

void __attribute__((weak, alias ("__empty_callback0"))) NRF905_CB_RXCOMPLETE(void);
void __attribute__((weak, alias ("__empty_callback0"))) NRF905_CB_RXINVALID(void);
void __attribute__((weak, alias ("__empty_callback0"))) NRF905_CB_TXCOMPLETE(void);
//void __attribute__((weak, alias ("__empty_callback0"))) NRF905_CB_AIRWAYBUSY(void);
//void __attribute__((weak, alias ("__empty_callback0"))) NRF905_CB_AIRWAYCLEAR(void);
void __attribute__((weak, alias ("__empty_callback0"))) NRF905_CB_ADDRMATCH(void);

#ifdef ARDUINO

#define SPI_SETTINGS	SPISettings(10000000, MSBFIRST, SPI_MODE0)

#if NRF905_INTERRUPTS != 0
// It's not possible to get the current interrupt enabled state in Arduino (SREG only works for AVR based Arduinos, and no way of getting attachInterrupt() status), so we use a counter thing instead
static volatile uint8_t isrState_local;

static void nRF905_SERVICE_DR(void);
#if NRF905_INTERRUPTS_AM
static void nRF905_SERVICE_AM(void);
#endif

#endif

#if NRF905_INTERRUPTS == 1 || NRF905_INT_SPI_COMMS == 1
static volatile uint8_t isrState;
static volatile uint8_t isrBusy; // Don't mess with global interrupts if we're inside an ISR

static inline uint8_t interrupt_off(void)
{
	if(!isrBusy)
	{
		noInterrupts();
		isrState++;
	}
	return 1;
}

static inline uint8_t interrupt_on(void)
{
	if(!isrBusy)
	{
		if(isrState > 0)
			isrState--;
		if(isrState == 0)
			interrupts();
	}
	return 0;
}
#endif

#endif

static inline uint8_t cselect(void)
{
//	spi_enable();
#ifdef ARDUINO
//	SPI.beginTransaction(SPI_SETTINGS);
#endif
	spiSelect();
	return 1;
}

static inline uint8_t cdeselect(void)
{
	spiDeselect();
#ifdef ARDUINO
//	SPI.endTransaction();
#endif
//	spi_disable();
	return 0;
}

// Can be in any mode to write registers, but standby or power down is recommended
#define STANDBY (STANDBY_ENTER())

#define CHIPSELECT(standby)	standby; \
							for(uint8_t _cs = cselect(); _cs; _cs = cdeselect())


/*
static inline uint8_t interrupt_off(void)
{
	nRF905_irq_off();
	return 1;
}

static inline uint8_t interrupt_on(void)
{
	nRF905_irq_on();
	return 0;
}
*/

#if NRF905_INTERRUPTS == 0 && NRF905_INT_SPI_COMMS == 0
#define NRF905_ATOMIC() ((void)(0));
#elif defined(ARDUINO)
#define NRF905_ATOMIC() for(uint8_t _cs2 = interrupt_off(); _cs2; _cs2 = interrupt_on())
#else
#define NRF905_ATOMIC()	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif

// When doing SPI comms with the radio or doing multiple commands we don't want the radio interrupt to mess it up.
uint8_t nRF905_irq_off()
{
#if NRF905_INTERRUPTS != 0

#ifdef ARDUINO
	detachInterrupt(digitalPinToInterrupt(NRF905_DR));
	#if NRF905_INTERRUPTS_AM
	detachInterrupt(digitalPinToInterrupt(NRF905_AM));
	#endif
	isrState_local++;
	return 0;
#else
	uint8_t origVal = NRF905_REG_EXTERNAL_INT_DR;
	NRF905_REG_EXTERNAL_INT_DR &= ~_BV(NRF905_BIT_EXTERNAL_INT_DR);
	#if NRF905_INTERRUPTS_AM
	NRF905_REG_EXTERNAL_INT_AM &= ~_BV(NRF905_BIT_EXTERNAL_INT_AM);
	#endif
	origVal = !!(origVal & _BV(NRF905_BIT_EXTERNAL_INT_DR));
	return origVal;
#endif

#else
	return 0;
#endif
}

void nRF905_irq_on(uint8_t origVal)
{
#if NRF905_INTERRUPTS != 0

#ifdef ARDUINO
	((void)(origVal));
	if(isrState_local > 0)
		isrState_local--;
	if(isrState_local == 0)
	{
		attachInterrupt(digitalPinToInterrupt(NRF905_DR), nRF905_SERVICE_DR, RISING);
		#if NRF905_INTERRUPTS_AM
		attachInterrupt(digitalPinToInterrupt(NRF905_AM), nRF905_SERVICE_AM, CHANGE);
		#endif
	}
#else
	if(origVal)
	{
		NRF905_REG_EXTERNAL_INT_DR |= _BV(NRF905_BIT_EXTERNAL_INT_DR);
		#if NRF905_INTERRUPTS_AM
		NRF905_REG_EXTERNAL_INT_AM |= _BV(NRF905_BIT_EXTERNAL_INT_AM);
		#endif
	}
#endif

#else
	((void)(origVal));
#endif
}

static uint8_t readConfigRegister(uint8_t reg)
{
	uint8_t val = 0;
	NRF905_ATOMIC()
	{
		CHIPSELECT()
		{
			spi_transfer_nr(NRF905_CMD_R_CONFIG | reg);
			val = spi_transfer(NRF905_CMD_NOP);
		}
	}
	return val;
}

static void writeConfigRegister(uint8_t reg, uint8_t val)
{
	NRF905_ATOMIC()
	{
		CHIPSELECT()
		{
			spi_transfer_nr(NRF905_CMD_W_CONFIG | reg);
			spi_transfer_nr(val);
		}
	}
}

static void setConfigReg1(uint8_t val, uint8_t mask, uint8_t reg)
{
	writeConfigRegister(reg, (readConfigRegister(NRF905_REG_CONFIG1) & mask) | val);
}

static void setConfigReg2(uint8_t val, uint8_t mask, uint8_t reg)
{
	writeConfigRegister(reg, (readConfigRegister(NRF905_REG_CONFIG2) & mask) | val);
}

static const uint8_t config[] PROGMEM = {
	NRF905_CMD_W_CONFIG,
	NRF905_CHANNEL,
	NRF905_AUTO_RETRAN | NRF905_LOW_RX | NRF905_PWR | NRF905_BAND | ((NRF905_CHANNEL>>8) & 0x01),
	(NRF905_ADDR_SIZE<<4) | NRF905_ADDR_SIZE,
	NRF905_PAYLOAD_SIZE, // RX payload size
	NRF905_PAYLOAD_SIZE, // TX payload size
	0xE7, 0xE7, 0xE7, 0xE7, // Default receive address
	NRF905_CRC | NRF905_CLK_FREQ | NRF905_OUTCLK
};

static noinline void defaultConfig(void)
{
	// Should be in standby mode

	NRF905_ATOMIC()
	{
		// Set control registers
		CHIPSELECT()
		{
			for(uint8_t i=0;i<sizeof(config);i++)
				spi_transfer_nr(pgm_read_byte(&((uint8_t*)config)[i]));
		}

		// Default transmit address
		CHIPSELECT()
		{
			spi_transfer_nr(NRF905_CMD_W_TX_ADDRESS);
			for(uint8_t i=0;i<4;i++)
				spi_transfer_nr(0xE7);
		}

		// Clear transmit payload
		CHIPSELECT()
		{
			spi_transfer_nr(NRF905_CMD_W_TX_PAYLOAD);
			for(uint8_t i=0;i<NRF905_MAX_PAYLOAD;i++)
				spi_transfer_nr(0x00);
		}

#if !NRF905_USE_PWR
		// Clear DR by reading receive payload
		CHIPSELECT()
		{
			spi_transfer_nr(NRF905_CMD_R_RX_PAYLOAD);
			for(uint8_t i=0;i<NRF905_MAX_PAYLOAD;i++)
				spi_transfer_nr(NRF905_CMD_NOP);
		}
#endif
	}
}

static void setAddress(uint32_t address, uint8_t cmd)
{
	NRF905_ATOMIC()
	{
		CHIPSELECT()
		{
			spi_transfer_nr(cmd);
			for(uint8_t i=0;i<4;i++)
				spi_transfer_nr(address>>(8 * i));
		}
	}
}

#if NEED_SW_STATUS_SUPPORT
static uint8_t readStatus(void)
{
	uint8_t status;
	NRF905_ATOMIC()
	{
		CHIPSELECT()
			status = spi_transfer(NRF905_CMD_NOP);
	}
	return status;
}
#endif

// Hardware: Data ready pin high
// Software: Data ready status bit set
#if !NRF905_INTERRUPTS
static uint8_t dataReady(void)
{
#if NRF905_DR_SW
	return (readStatus() & (1<<NRF905_STATUS_DR));
#elif defined(ARDUINO)
	return digitalRead(NRF905_DR);
#else
	return (DR_PORT & _BV(DR_BIT));
#endif
}
#endif

// Hardware: Address match pin high
// Software: Address match status bit set
static uint8_t addressMatched(void)
{
#if NRF905_AM_SW
	return (readStatus() & (1<<NRF905_STATUS_AM));
#elif defined(ARDUINO)
	return digitalRead(NRF905_AM);
#else
	return (AM_PORT & _BV(AM_BIT));
#endif
}

void nRF905_init()
{
#ifdef ARDUINO
	digitalWrite(NRF905_CSN, HIGH);
	pinMode(NRF905_CSN, OUTPUT);

	pinMode(NRF905_TRX_EN, OUTPUT);
	pinMode(NRF905_TX_EN, OUTPUT);

#if NRF905_USE_PWR
	pinMode(NRF905_PWR_MODE, OUTPUT);
#endif

	SPI.begin();
//	SPI.setClockDivider(SPI_CLOCK_DIV2);  // TODO CHECK THIS! Max SPI clock is 10MHz, but DIV2 will be 42MHz on the Due!
//	SPI.usingInterrupt(digitalPinToInterrupt(NRF905_DR));
//#if NRF905_INTERRUPTS_AM
//	SPI.usingInterrupt(digitalPinToInterrupt(NRF905_AM));
//#endif
#else
	spiDeselect();
	CSN_DDR |= _BV(CSN_BIT);	

	TRX_EN_DDR |= _BV(TRX_EN_BIT);
	TX_EN_DDR |= _BV(TX_EN_BIT);

#if NRF905_USE_PWR
	PWR_MODE_DDR |= _BV(PWR_MODE_BIT);
#endif

	spi_init();
#endif

	POWER_DOWN();
	STANDBY_ENTER();
	MODE_RX();
	delay_ms(3);
	defaultConfig();

#if NRF905_INTERRUPTS
	// Set interrupts
	#if !defined(ARDUINO)
	NRF905_REG_EXTERNAL_INT_CTL_DR |= NRF905_BIT_EXTERNAL_INT_CTL_DR; // Trigger on rising
	#if NRF905_INTERRUPTS_AM
	NRF905_REG_EXTERNAL_INT_CTL_AM |= NRF905_BIT_EXTERNAL_INT_CTL_AM; // Trigger on change
	#endif
	#endif
	nRF905_irq_on(1);
#endif
}

void nRF905_setChannel(uint16_t channel)
{
	if(channel > 511)
		channel = 511;

	NRF905_ATOMIC()
	{
		uint8_t reg = (readConfigRegister(NRF905_REG_CONFIG1) & NRF905_MASK_CHANNEL) | (channel>>8);

		CHIPSELECT()
		{
			spi_transfer_nr(NRF905_CMD_W_CONFIG | NRF905_REG_CHANNEL);
			spi_transfer_nr(channel);
			spi_transfer_nr(reg);
		}
	}
}

void nRF905_setBand(nRF905_band_t band)
{
	NRF905_ATOMIC()
	{
		uint8_t reg = (readConfigRegister(NRF905_REG_CONFIG1) & NRF905_MASK_BAND) | band;

		CHIPSELECT()
		{
			spi_transfer_nr(NRF905_CMD_W_CONFIG | NRF905_REG_CONFIG1);
			spi_transfer_nr(reg);
		}
	}
}

void nRF905_setAutoRetransmit(nRF905_auto_retran_t val)
{
	setConfigReg1(val, NRF905_MASK_AUTO_RETRAN, NRF905_REG_AUTO_RETRAN);
}

void nRF905_setLowRxPower(nRF905_low_rx_t val)
{
	setConfigReg1(val, NRF905_MASK_LOW_RX, NRF905_REG_LOW_RX);
}

void nRF905_setTransmitPower(nRF905_pwr_t val)
{
	setConfigReg1(val, NRF905_MASK_PWR, NRF905_REG_PWR);
}

void nRF905_setCRC(nRF905_crc_t val)
{
	setConfigReg2(val, NRF905_MASK_CRC, NRF905_REG_CRC);
}

void nRF905_setClockOut(nRF905_outclk_t val)
{
	setConfigReg2(val, NRF905_MASK_OUTCLK, NRF905_REG_OUTCLK);
}

void nRF905_setPayloadSize(uint8_t size)
{
	NRF905_ATOMIC()
	{
		CHIPSELECT()
		{
			if(size > NRF905_MAX_PAYLOAD)
				size = NRF905_MAX_PAYLOAD;

			spi_transfer_nr(NRF905_CMD_W_CONFIG | NRF905_REG_RX_PAYLOAD_SIZE);
			spi_transfer_nr(size);
			spi_transfer_nr(size);
		}
	}
}

void nRF905_setAddressSize(nRF905_addr_size_t size)
{
	CHIPSELECT()
	{
		spi_transfer_nr(NRF905_CMD_W_CONFIG | NRF905_REG_ADDR_WIDTH);
		spi_transfer_nr((size<<4) | size);
	}
}

uint8_t nRF905_receiveBusy()
{
	return addressMatched();
}

uint8_t nRF905_airwayBusy()
{
#ifdef ARDUINO
	return digitalRead(NRF905_CD);
#else
	return (CD_PORT & _BV(CD_BIT));
#endif
}

void nRF905_setListenAddress(uint32_t address)
{
	setAddress(address, NRF905_CMD_W_CONFIG | NRF905_REG_RX_ADDRESS);
}

uint8_t nRF905_TX(uint32_t sendTo, void* data, uint8_t len, nRF905_nextmode_t nextMode)
{
	// TODO check DR is low?

	NRF905_NO_INTERRUPT()
	{
#if NRF905_COLLISION_AVOID
		if(nRF905_airwayBusy())
			return 0;
#endif

		setAddress(sendTo, NRF905_CMD_W_TX_ADDRESS);

		// Load new payload
		if(data != NULL)
		{
			NRF905_ATOMIC()
			{
				CHIPSELECT()
				{
					spi_transfer_nr(NRF905_CMD_W_TX_PAYLOAD);
					for(uint8_t i=0;i<len;i++)
						spi_transfer_nr(((uint8_t*)data)[i]);
				}
			}
		}

		if(!POWERED_UP())
		{
			STANDBY_ENTER();
			POWER_UP();
			delay_ms(3);
		}

#if NRF905_COLLISION_AVOID
		if(nRF905_airwayBusy())
			return 0;
#endif

		// Put into transmit mode
		MODE_TX();

		// Pulse standby pin to start transmission
		STANDBY_LEAVE();

		if(nextMode == NRF905_NEXTMODE_RX)
		{
			// The datasheets says that the radio can switch straight to RX mode after
			// a transmission is complete by clearing TX_EN while transmitting, but
			// if this is done within ~700us the transmission seems to get corrupt.
			delay_us(700);
			MODE_RX();
		}
		else if(nextMode == NRF905_NEXTMODE_STANDBY)
		{
			delay_us(14);
			STANDBY_ENTER();
		}
		// else NRF905_NEXTMODE_TX
	}

	return 1;
}

void nRF905_RX()
{
	NRF905_NO_INTERRUPT()
	{
		MODE_RX();
		STANDBY_LEAVE();
		POWER_UP();
	}
}

void nRF905_read(void* data, uint8_t len)
{
	if(len > NRF905_MAX_PAYLOAD)
		len = NRF905_MAX_PAYLOAD;

	NRF905_ATOMIC()
	{
		CHIPSELECT()
		{
			spi_transfer_nr(NRF905_CMD_R_RX_PAYLOAD);

			// Get received payload
			for(uint8_t i=0;i<len;i++)
				((uint8_t*)data)[i] = spi_transfer(NRF905_CMD_NOP);

			// Must make sure all of the payload has been read, otherwise DR never goes low
			//uint8_t remaining = NRF905_MAX_PAYLOAD - len;
			//while(remaining--)
			//	spi_transfer_nr(NRF905_CMD_NOP);
		}
	}
}

void nRF905_powerDown()
{
	POWER_DOWN();
}

void nRF905_powerUp()
{
	uint8_t wasPoweredUp = POWERED_UP();
	STANDBY_ENTER();
	POWER_UP();
	if(!wasPoweredUp)
		delay_ms(3);
}

void nRF905_standby()
{
	STANDBY_ENTER();
	POWER_UP();
}

void nRF905_getConfigRegisters(void* regs)
{
	NRF905_ATOMIC()
	{
		CHIPSELECT()
		{
			spi_transfer_nr(NRF905_CMD_R_CONFIG);
			for(uint8_t i=0;i<NRF905_REGISTER_COUNT;i++)
				((uint8_t*)regs)[i] = spi_transfer(NRF905_CMD_NOP);
		}
	}
}

#if !NRF905_INTERRUPTS

void nRF905_SERVICE()
{
	static uint8_t invalidPacket;
	static uint8_t rxComplete;
	static uint8_t addrMatch;
	static uint8_t txComplete;

	uint8_t state_DR = dataReady();
	uint8_t state_AM = addressMatched();
	
	if(state_AM)
	{
		txComplete = 0;
		invalidPacket = 0;

		if(!addrMatch)
		{
			NRF905_CB_ADDRMATCH();
			addrMatch = 1;
		}

		if(state_DR && !rxComplete)
		{
			NRF905_CB_RXCOMPLETE();
			rxComplete = 1;
		}
	}
	else
	{
		if(!invalidPacket)
		{
			if(!rxComplete)
				NRF905_CB_RXINVALID();
			invalidPacket = 1;
		}

		if(state_DR && !txComplete)
		{
			NRF905_CB_TXCOMPLETE();
			txComplete = 1;
		}

		addrMatch = 0;
		rxComplete = 0;
	}
}

#else

static volatile uint8_t validPacket;

#ifdef ARDUINO
static void nRF905_SERVICE_DR()
#else
ISR(NRF905_INT_VECTOR_DR)
#endif
{
#if defined(ARDUINO) && (NRF905_INTERRUPTS == 1 || NRF905_INT_SPI_COMMS == 1)
	isrBusy = 1;
#endif
	// If DR && AM = RX new packet
	// If DR && !AM = TX finished

	if(addressMatched())
	{
		validPacket = 1;
		NRF905_CB_RXCOMPLETE();
	}
	else
		NRF905_CB_TXCOMPLETE();

#if defined(ARDUINO) && (NRF905_INTERRUPTS == 1 || NRF905_INT_SPI_COMMS == 1)
	isrBusy = 0;
#endif
}

#if NRF905_INTERRUPTS_AM

#ifdef ARDUINO
static void nRF905_SERVICE_AM()
#else
ISR(NRF905_INT_VECTOR_AM)
#endif
{
#if defined(ARDUINO) && (NRF905_INTERRUPTS == 1 || NRF905_INT_SPI_COMMS == 1)
	isrBusy = 1;
#endif
	// If AM goes HIGH then LOW without DR going HIGH then we got a bad packet

	if(addressMatched())
		NRF905_CB_ADDRMATCH();
	else if(!validPacket)
		NRF905_CB_RXINVALID();
	validPacket = 0;

#if defined(ARDUINO) && (NRF905_INTERRUPTS == 1 || NRF905_INT_SPI_COMMS == 1)
	isrBusy = 0;
#endif
}

#endif

#endif
