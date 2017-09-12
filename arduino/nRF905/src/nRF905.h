/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#ifndef NRF905_H_
#define NRF905_H_

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include <stdint.h>

#include "nRF905_config.h"

/**
* @brief Available modes after transmission complete.
*/
typedef enum
{
	NRF905_NEXTMODE_STANDBY, ///< Standby mode
	NRF905_NEXTMODE_RX, ///< Receive mode
	NRF905_NEXTMODE_TX ///< Transmit mode (will auto-retransmit if ::NRF905_AUTO_RETRAN is ::NRF905_AUTO_RETRAN_ENABLE, otherwise will transmit a carrier wave with no data)
} nRF905_nextmode_t;

/**
* @brief Frequency bands.
*/
typedef enum
{
// NOTE:
// When using NRF905_BAND_868 and NRF905_BAND_915 for calculating channel (NRF905_CALC_CHANNEL(f, b)) they should be value 0x01,
// but when using them for setting registers their value should be 0x02.
// They're defined as 0x02 here so when used for calculating channel they're right shifted by 1

	NRF905_BAND_433 = 0x00,	///< 433MHz band
	NRF905_BAND_868 = 0x02,	///< 868/915MHz band
	NRF905_BAND_915 = 0x02	///< 868/915MHz band
} nRF905_band_t;

/**
* @brief Output power (n means negative, n10 = -10).
*/
typedef enum
{
	NRF905_PWR_n10 = 0x00,	///< -10dBm = 100uW
	NRF905_PWR_n2 = 0x04,	///< -2dBm = 631uW
	NRF905_PWR_6 = 0x08,	///< 6dBm = 4mW
	NRF905_PWR_10 = 0x0C	///< 10dBm = 10mW
} nRF905_pwr_t;

/**
* @brief Save a few mA by reducing receive sensitivity.
*/
typedef enum
{
	NRF905_LOW_RX_DISABLE = 0x00,	///< Disable low power receive
	NRF905_LOW_RX_ENABLE = 0x10		///< Enable low power receive
} nRF905_low_rx_t;

/**
* @brief Auto re-transmit options.
*/
typedef enum
{
	NRF905_AUTO_RETRAN_DISABLE = 0x00,	///< Disable auto re-transmit
	NRF905_AUTO_RETRAN_ENABLE = 0x20	///< Enable auto re-transmit
} nRF905_auto_retran_t;

/**
* @brief Output a clock signal on pin 3 of IC.
*/
typedef enum
{
	NRF905_OUTCLK_DISABLE = 0x00,	///< Disable output clock
	NRF905_OUTCLK_4MHZ = 0x04,		///< 4MHz clock
	NRF905_OUTCLK_2MHZ = 0x05,		///< 2MHz clock
	NRF905_OUTCLK_1MHZ = 0x06,		///< 1MHz clock
	NRF905_OUTCLK_500KHZ = 0x07,	///< 500KHz clock (default)
} nRF905_outclk_t;

/**
* @brief CRC Checksum.
*
* The CRC is calculated across the address (SYNC word) and payload
*/
typedef enum
{
	NRF905_CRC_DISABLE = 0x00,	///< Disable CRC
	NRF905_CRC_8 = 0x40,		///< 8bit CRC (Don't know what algorithm is used for this one)
	NRF905_CRC_16 = 0xC0,		///< 16bit CRC (CRC16-CCITT-FALSE (0xFFFF))
} nRF905_crc_t;

/**
* @brief Address size.
*
* This is actually used as the SYNC word
*/
typedef enum
{
	NRF905_ADDR_SIZE_1 = 0x01,	///< 1 byte (not recommended, a lot of false invalid packets will be received)
	NRF905_ADDR_SIZE_4 = 0x04,	///< 4 bytes
} nRF905_addr_size_t;

// TODO remove?
// Setting options
//#define NRF905_BAND_433			0x00
//#define NRF905_BAND_868			0x02
//#define NRF905_BAND_915			0x02
//#define NRF905_PWR_n10			0x00
//#define NRF905_PWR_n2			0x04
//#define NRF905_PWR_6			0x08
//#define NRF905_PWR_10			0x0C
//#define NRF905_LOW_RX_ENABLE	0x10
//#define NRF905_LOW_RX_DISABLE	0x00
//#define NRF905_AUTO_RETRAN_ENABLE	0x20
//#define NRF905_AUTO_RETRAN_DISABLE	0x00
//#define NRF905_OUTCLK_DISABLE	0x00
//#define NRF905_OUTCLK_4MHZ		0x04
//#define NRF905_OUTCLK_2MHZ		0x05
//#define NRF905_OUTCLK_1MHZ		0x06
//#define NRF905_OUTCLK_500KHZ	0x07
//#define NRF905_CRC_DISABLE		0x00
//#define NRF905_CRC_8			0x40
//#define NRF905_CRC_16			0xC0
//#define NRF905_ADDR_SIZE_1		0x01
//#define NRF905_ADDR_SIZE_4		0x04

#define NRF905_MAX_PAYLOAD		32 ///< Maximum payload size
#define NRF905_REGISTER_COUNT	10 ///< Configuration register count
#define NRF905_DEFAULT_RXADDR	0xE7E7E7E7 ///< Default receive address
#define NRF905_DEFAULT_TXADDR	0xE7E7E7E7 ///< Default transmit/destination address

#define NRF905_CALC_CHANNEL(f, b)	((((f) / (1 + (b>>1))) - 422400000UL) / 100000UL) ///< Workout channel from frequency & band

#if defined(__cplusplus)
extern "C" {
#endif

/**
* @brief Initialise, must be called before anything else!
*
* @return (none)
*/
void nRF905_init(void);

/**
* @brief Channel to listen and transmit on
*
* 433MHz band: Channel 0 is 422.4MHz up to 511 which is 473.5MHz (Each channel is 100KHz apart)
*
* 868/915MHz band: Channel 0 is 844.8MHz up to 511 which is 947MHz (Each channel is 200KHz apart)
*
* @param [channel] The channel (0 - 511)
* @return (none)
*
* @see ::nRF905_setBand()
*/
void nRF905_setChannel(uint16_t channel);

/**
* @brief Frequency band
*
* @param [band] Frequency band, see ::nRF905_band_t
* @return (none)
*
* @see ::nRF905_setChannel() ::nRF905_band_t
*/
void nRF905_setBand(nRF905_band_t band);

/**
* @brief Set auto retransmit
*
* If next mode is set to ::NRF905_NEXTMODE_TX when calling ::nRF905_TX() and auto-retransmit is enabled then it will constantly retransmit the payload, otherwise a carrier wave with no data will be transmitted instead (kinda useless).\n
* Transmission will continue until the radio is put into standby, power down or RX mode.
*
* Can be useful in areas with lots of interference, but you'll need to make sure you can differentiate between re-transmitted packets and new packets (like an ID number).
*
* Other transmissions will be blocked if collision avoidance is enabled.
*
* @param [val] Enable/disable auto retransmit, see ::nRF905_auto_retran_t
* @return (none)
*/
void nRF905_setAutoRetransmit(nRF905_auto_retran_t val);

/**
* @brief Set low power receive
*
* @param [val] Enable/disable low power receive, see ::nRF905_low_rx_t
* @return (none)
*/
void nRF905_setLowRxPower(nRF905_low_rx_t val);

/**
* @brief Set output power
*
* @param [val] Output power level, see ::nRF905_pwr_t
* @return (none)
*/
void nRF905_setTransmitPower(nRF905_pwr_t val);

/**
* @brief Set CRC
*
* @param [val] CRC Type, see ::nRF905_crc_t
* @return (none)
*/
void nRF905_setCRC(nRF905_crc_t val);

/**
* @brief Set clock output
*
* @param [val] Clock out frequency, see ::nRF905_outclk_t
* @return (none)
*/
void nRF905_setClockOut(nRF905_outclk_t val);

/**
* @brief Payload size
*
* @param [size] Payload size (1 - 32)
* @return (none)
*
* @see ::NRF905_MAX_PAYLOAD
*/
void nRF905_setPayloadSize(uint8_t size);

/**
* @brief Address size
*
* @param [size] Address size, see ::nRF905_addr_size_t
* @return (none)
*/
void nRF905_setAddressSize(nRF905_addr_size_t size);

/**
* @brief See if the attach match is asserted
*
* @return 1 if currently receiving payload or payload is ready to be read, otherwise 0
*/
uint8_t nRF905_receiveBusy(void);

/**
* @brief See if airway is busy (carrier detect pin asserted).
*
* @return 1 if other transmissions detected, otherwise 0
*/
uint8_t nRF905_airwayBusy(void);

/**
* @brief Set address to listen to
*
* @note From the datasheet: Each byte within the address should be unique. Repeating bytes within the address reduces the effectiveness of the address and increases its susceptibility to noise which increases the packet error rate. The address should also have several level shifts (that is, 10101100) reducing the statistical effect of noise and the packet error rate.
* @param [address] The address, a 32 bit integer (default address is 0xE7E7E7E7)
* @return (none)
*/
void nRF905_setListenAddress(uint32_t address);

/**
* @brief Begin a transmission
*
* If the radio is still transmitting then the the payload and address will be updated as it is being sent, this means the payload on the receiving end may contain old and new data.\n
* This also means that a node may receive part of a payload that was meant for a node with a different address.\n
* Use the ::NRF905_CB_TXCOMPLETE callback to set a flag or something to ensure the transmission is complete before sending another payload.
*
* If the radio is in power down mode then this function will take an additional 3ms to complete.
* If 3ms is too long then call ::nRF905_standby(), do whatever you need to do for at least 3ms then call ::nRF905_TX().
*
* If \p nextMode is set to ::NRF905_NEXTMODE_RX then this function will take an additional 700us to complete.\n
* If 700us is too long then set \p nextMode to ::NRF905_NEXTMODE_STANDBY and call ::nRF905_RX() in the ::NRF905_CB_TXCOMPLETE callback instead.
*
* If \p data is NULL and/or \p len is 0 then the payload will not be modified, whatever was previously transmitted will be sent again to the \p sendTo address.
*
* For the collision avoidance to work the radio should be in RX mode for around 5ms before attempting to transmit.
*
* @param [sendTo] Address to send the payload to
* @param [data] The data
* @param [len] Data length (max ::NRF905_MAX_PAYLOAD)
* @param [nextMode] What mode to enter once the transmission is complete, see ::nRF905_nextmode_t
* @return 0 if other transmissions are going on and collision avoidance is enabled, 1 if transmission has successfully began
*/
uint8_t nRF905_TX(uint32_t sendTo, void* data, uint8_t len, nRF905_nextmode_t nextMode);

/**
* @brief Enter receive mode.
*
* If the radio is currently transmitting then receive mode will be entered once it has finished.
* This function will also automatically power up the radio and leave standby mode.
*
* @return (none)
*/
void nRF905_RX(void);

/**
* @brief Get received payload.
*
* This function can be called multiple times to read a few bytes at a time.
* The payload is cleared when the radio enters power down mode or leaves standby and enters RX mode.
* The radio will not receive anymore data until all of the payload has been read or cleared.
*
* @param [data] Buffer for the data
* @param [len] How many bytes to get
* @return (none)
*/
void nRF905_read(void* data, uint8_t len);

/**
* @brief Sleep.
*
* This also clears the RX payload.
*
* @return (none)
*/
void nRF905_powerDown(void);

/**
* @brief Enter standby mode.
*
* Will take 3ms to complete if the radio was in power down mode.
* ::nRF905_standby() does the same thing, but without the delay.
* There must be a 3ms delay between powering up and beginning a transmission.
*
* @return (none)
*/
void nRF905_powerUp(void);

/**
* @brief Enter standby mode.
*
* Similar to ::nRF905_powerUp() but without any delays.
* There must be a 3ms delay between powering up and beginning a transmission.
*
* @return (none)
*/
void nRF905_standby(void);

/**
* @brief Read configuration registers into byte array of ::NRF905_REGISTER_COUNT elements, mainly for debugging.
*
* @return (none)
*/
void nRF905_getConfigRegisters(void*);

/**
* @brief If interrupts are disabled (::NRF905_INTERRUPTS in nRF905_config.h) then this function should be called as often as possible to process any events
*
* If the radio seems to stop receiving new data then you're probably not calling this often enough (at least every few milliseconds).
*
* @return (none)
*/
#if DOXYGEN || NRF905_INTERRUPTS == 0
void nRF905_SERVICE(void);
#else
#define nRF905_SERVICE() ((void)(0))
#endif

/**
* @brief When using interrupts use this to disable them for the nRF905
*
* Ideally you should wrap sensitive sections with ::NRF905_NO_INTERRUPT() instead, as it automatically deals with this function and ::nRF905_irq_on()
*
* @see ::nRF905_irq_on() and ::NRF905_NO_INTERRUPT()
* @return The previous interrupt status; 1 if interrupt was enabled, 0 if it was already disabled
*/
uint8_t nRF905_irq_off(void);

/**
* @brief When using interrupts use this to re-enable them for the nRF905
*
* Ideally you should wrap sensitive sections with ::NRF905_NO_INTERRUPT() instead, as it automatically deals with this function and ::nRF905_irq_off()
*
* @see ::nRF905_irq_off() and ::NRF905_NO_INTERRUPT()
* @param [origVal] The original interrupt status returned from ::nRF905_irq_off()
* @return (none)
*/
void nRF905_irq_on(uint8_t origVal);

#if DOXYGEN || NRF905_INTERRUPTS != 0

#if !defined(DOXYGEN)
static inline void _nRF905_iRestore(const  uint8_t *__s)
{
	nRF905_irq_on(*__s);
	__asm__ volatile ("" ::: "memory");
}
#endif

/**
* @brief Disable nRF905 interrupts for code inside this block
*
* When communicating with other SPI devices on the same bus as the radio then you should wrap those sections in a ::NRF905_NO_INTERRUPT() block, this will stop the nRF905 interrupt from running and trying to use the bus at the same time.
* This macro is based on the code from avr/atomic.h, and wraps the ::nRF905_irq_off() and ::nRF905_irq_on() functions instead of messing with global interrupts. It is safe to return, break or continue inside an ::NRF905_NO_INTERRUPT() block.
*
* Example:
*
* nRF905_RX();\n
* NRF905_NO_INTERRUPT()\n
* {\n
* 	OLED.write("blah", 2, 10); // Communicate with SPI OLED display\n
* }\n
*/
#define NRF905_NO_INTERRUPT() \
	for(uint8_t nrf905_irq __attribute__((__cleanup__(_nRF905_iRestore))) = nRF905_irq_off(), \
	nrf905_tmp = 1; nrf905_tmp ; nrf905_tmp = 0)

#else
#define NRF905_NO_INTERRUPT() ((void)(0));
#endif


#if defined(__cplusplus)
}
#endif

#endif /* NRF905_H_ */
