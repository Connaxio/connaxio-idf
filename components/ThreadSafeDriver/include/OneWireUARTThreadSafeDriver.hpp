/*
 * OneWireUARTThreadSafeDriver.hpp
 *
 *  Created on: Mar. 23, 2022
 *      Author: malalonde
 */

#ifndef ONEWIREUARTTHREADSAFEDRIVER_HPP_
#define ONEWIREUARTTHREADSAFEDRIVER_HPP_

#include <vector>
#include <array>

#include "esp_err.h"

#include "UARTThreadSafeDriver.hpp"

class OneWireUARTThreadSafeDriver {
public:
	OneWireUARTThreadSafeDriver(gpio_num_t tx_io_num, gpio_num_t rx_io_num,
			uart_port_t uart_num = UART_NUM_1);
	~OneWireUARTThreadSafeDriver();

	/* \brief Search for a device of the specified family ID.
	 * \param family_id 	The ID of the target device's family
	 * \param search_begin_index	Index of the first device to check.
	 * \param address	Pointer to the address of the found device, or NULL
	 *
	 * \return The index of the found device, or -1 if no more devices found
	 * */
	std::array<uint8_t, 8> findFamilyDevice(uint8_t family_id, uint8_t search_begin_index,
			std::vector<std::array<uint8_t, 8>> addresses);

	/*! \brief  Sends bytes of data on the 1-Wire(R) bus(es).
	 *
	 *  This function automates the task of sending a complete byte
	 *  of data on the 1-Wire bus(es).
	 *
	 *  \param  address	The device's 8 bytes address
	 *  \param  code    The device command code
	 *  \param  data    The data to send on the bus(es).
	 *  \param  pins    A bitmask of the buses to send the data to.
	 */

	esp_err_t write(std::array<uint8_t, 8> address, uint8_t code, uint8_t data[], uint8_t length);

	/* \brief Write to all devices on the bus with the SKIP ROM command code
	 *
	 */
	esp_err_t writeAll(uint8_t code, uint8_t data[], uint8_t length);

	esp_err_t read(std::array<uint8_t, 8> address, uint8_t code, uint8_t buffer[], uint8_t length);

	esp_err_t readAny(uint8_t code, uint8_t buffer[], uint8_t length);

	/*! \brief  Sends the READ ROM command and reads back the ROM id.
	 * Can only be used if there is guaranteed to be only 1 device.
	 *
	 *  \param  romValue    A pointer where the id will be placed.
	 *  \param  pin     A bitmask of the bus to read from.
	 */
	esp_err_t readROM(std::array<uint8_t, 8> address);

	/*! \brief  Perform a 1-Wire search
	 *
	 *  This function shows how the searchRom function can be used to
	 *  discover all slaves on the bus. It will also CRC check the 64 bit
	 *  identifiers.
	 *
	 *  \param addresses	The output vector of addresses
	 */
	esp_err_t searchBus(std::vector<std::array<uint8_t, 8>> &addresses);

	/*! \brief  Read a bit from the bus. (Polled UART DRIVER)
	 *
	 *  Generates the waveform for reception of a bit on the 1-Wire(R) bus(es).
	 *
	 *  \return The value read from the bus (0 or 1).
	 */
	esp_err_t readBit(uint8_t &bit);

	bool hasValidROM(std::array<uint8_t, 8> address);

	/*! \brief  Compute the CRC8 value of a data set.
	 *
	 *  This function will compute the CRC8 or DOW-CRC of inData using seed
	 *  as inital value for the CRC.
	 *
	 *  \param  inData  One byte of data to compute CRC from.
	 *
	 *  \param  seed    The starting value of the CRC.
	 *
	 *  \return The CRC8 of inData with seed as initial value.
	 *
	 *  \note   Setting seed to 0 computes the crc8 of the inData.
	 *
	 *  \note   Constantly passing the return value of this function
	 *          As the seed argument computes the CRC8 value of a
	 *          longer string of data.
	 */
	static uint8_t computeCRC8(unsigned char data_in, unsigned char seed);

	/*! \brief  Compute the CRC16 value of a data set.
	 *
	 *  This function will compute the CRC16 of inData using seed
	 *  as inital value for the CRC.
	 *
	 *  \param  inData  One byte of data to compute CRC from.
	 *
	 *  \param  seed    The starting value of the CRC.
	 *
	 *  \return The CRC16 of inData with seed as initial value.
	 *
	 *  \note   Setting seed to 0 computes the crc16 of the inData.
	 *
	 *  \note   Constantly passing the return value of this function
	 *          As the seed argument computes the CRC16 value of a
	 *          longer string of data.
	 */
	static uint32_t computeCRC16(unsigned char data_in, unsigned int seed);

	/*! \brief  Calculate and check the CRC of a 64 bit ROM identifier.
	 *
	 *  This function computes the CRC8 value of the first 56 bits of a
	 *  64 bit identifier. It then checks the calculated value against the
	 *  CRC value stored in ROM.
	 *
	 *  \param  romvalue    A pointer to an array holding a 64 bit identifier.
	 *
	 *  \retval ESP_OK      The CRC's matched.
	 *  \retval OWI_CRC_ERROR   There was a discrepancy between the calculated and the stored CRC.
	 */
	static esp_err_t checkOneWireCRC(uint8_t data[], uint8_t length);

	uint8_t getUARTNum();

private:
	UARTThreadSafeDriver *m_uart_driver;

	/*! \brief  Sends the SEARCH ROM command and returns 1 id found on the
	 *          1-Wire(R) bus.
	 *
	 *  \param  bitPattern      A pointer to an 8 byte char array where the
	 *                          discovered identifier will be placed. When
	 *                          searching for several slaves, a copy of the
	 *                          last found identifier should be supplied in
	 *                          the array, or the search will fail.
	 *
	 *  \param  lastDeviation   The bit position where the algorithm made a
	 *                          choice the last time it was run. This argument
	 *                          should be 0 when a search is initiated. Supplying
	 *                          the return argument of this function when calling
	 *                          repeatedly will go through the complete slave
	 *                          search.
	 *
	 *  \param  pin             A bit-mask of the bus to perform a ROM search on.
	 *
	 *  \return The last bit position where there was a discrepancy between slave
	 *          addresses the last time this function was run. Returns
	 *          OWI_ROM_SEARCH_FAILED if an error was detected (e.g. a device was
	 *          connected to the bus during the search), or OWI_ROM_SEARCH_FINISHED
	 *          when there are no more devices to be discovered.
	 *
	 *  \note   See main.c for an example of how to utilize this function.
	 */
	esp_err_t searchROM(uint8_t &last_deviation, std::array<uint8_t, 8> &address);

	/*! \brief  Sends one byte of data on the 1-Wire(R) bus(es).
	 *
	 *  This function automates the task of sending a complete byte
	 *  of data on the 1-Wire bus(es).
	 *
	 *  \param  data    The data to send on the bus(es).
	 *  \param  pins    A bitmask of the buses to send the data to.
	 */
	esp_err_t writeByte(uint8_t data);

	/*! \brief Write a '1' bit to the bus. (Polled UART DRIVER)
	 *
	 *  Generates the waveform for transmission of a '1' bit on the 1-Wire
	 *  bus.
	 */
	esp_err_t write1();

	/*! \brief  Write a '0' to the bus. (Polled UART DRIVER)
	 *
	 *  Generates the waveform for transmission of a '0' bit on the 1-Wire(R)
	 *  bus.
	 */
	esp_err_t write0();

	/*! \brief  Send a Reset signal and listen for Presence signal. (Polled
	 *  UART DRIVER)
	 *
	 *  Generates the waveform for transmission of a Reset pulse on the
	 *  1-Wire(R) bus and listens for presence signals.
	 *
	 *  \return non zero value when a presence signal was detected.
	 */
	esp_err_t detectPresence(uint8_t &presence);

};

#endif /* ONEWIREUARTTHREADSAFEDRIVER_HPP_ */
