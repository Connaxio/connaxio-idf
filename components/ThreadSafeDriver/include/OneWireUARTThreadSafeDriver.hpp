/*
 * OneWireUARTThreadSafeDriver.hpp
 *
 *  Created on: Mar. 23, 2022
 *      Author: malalonde
 */

#ifndef ONEWIREUARTTHREADSAFEDRIVER_HPP_
#define ONEWIREUARTTHREADSAFEDRIVER_HPP_

#include "UARTThreadSafeDriver.hpp"

class OneWireUARTThreadSafeDriver {
public:
	OneWireUARTThreadSafeDriver(uart_port_t uart_num, gpio_num_t tx_io_num, gpio_num_t rx_io_num);
	virtual ~OneWireUARTThreadSafeDriver();

	/* \brief Search for a device of the specified family ID.
	 * \param family_id 	The ID of the target device's family
	 * \param search_begin_index	Index of the first device to check.
	 * \param address	Pointer to the address of the found device, or NULL
	 *
	 * \return The index of the found device, or -1 if no more devices found
	 * */
	int8_t findFamilyDevice(uint8_t family_id, uint8_t search_begin_index, uint8_t (&address)[8]);

	/*! \brief  Sends one byte of data on the 1-Wire(R) bus(es).
	 *
	 *  This function automates the task of sending a complete byte
	 *  of data on the 1-Wire bus(es).
	 *
	 *  \param  data    The data to send on the bus(es).
	 *  \param  pins    A bitmask of the buses to send the data to.
	 */
	esp_err_t writeByte(uint8_t data);

	/*! \brief  Receives one byte of data from the 1-Wire(R) bus.
	 *
	 *  This function automates the task of receiving a complete byte
	 *  of data from the 1-Wire bus.
	 *
	 *  \param  pin     A bitmask of the bus to read from.
	 *  \return     The byte read from the bus.
	 */
	esp_err_t readByte(uint8_t &data);

	/*! \brief  Sends the SKIP ROM command to the 1-Wire bus(es).
	 * Used to send a command to all devices at once.
	 *
	 *  \param  pins    A bitmask of the buses to send the SKIP ROM command to.
	 */
	esp_err_t skipROM();

	/*! \brief  Sends the READ ROM command and reads back the ROM id.
	 * Can only be used if there is guaranteed to be only 1 device.
	 *
	 *  \param  romValue    A pointer where the id will be placed.
	 *  \param  pin     A bitmask of the bus to read from.
	 */
	esp_err_t readROM(uint8_t (&address)[8]);

	/*! \brief  Sends the MATCH ROM command and the ROM id to match against.
	 *
	 *  \param  romValue    A pointer to the ID to match against.
	 *  \param  pins    A bitmask of the buses to perform the MATCH ROM command on.
	 */
	esp_err_t matchROM(uint8_t (&address)[8]);

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
	esp_err_t searchROM(uint8_t &lastDeviation, uint8_t (&address)[8]);

	/*! \brief  Perform a 1-Wire search
	 *
	 *  This function shows how the searchRom function can be used to
	 *  discover all slaves on the bus. It will also CRC check the 64 bit
	 *  identifiers.
	 */
	esp_err_t searchBus();

	/*! \brief  Read a bit from the bus. (Polled UART DRIVER)
	 *
	 *  Generates the waveform for reception of a bit on the 1-Wire(R) bus(es).
	 *
	 *  \return The value read from the bus (0 or 1).
	 */
	esp_err_t readBit(uint8_t &bit);

	bool hasValidROM(uint8_t (&address)[8]);

	uint8_t getUARTNum();

private:
	UARTThreadSafeDriver *m_uart_driver;
	uint8_t m_num_devices;
	uint8_t (*m_addresses)[8];

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
	static esp_err_t checkROMCRC(uint8_t (&address)[8]);

	/*! \brief  Write and read one bit to/from the 1-Wire bus. (Polled UART driver)
	 *
	 *  Writes one bit to the bus and returns the value read from the bus.
	 *
	 *  \param  outValue    The value to transmit on the bus.
	 *
	 *  \return The value received by the UART from the bus.
	 */
	esp_err_t touchBit(uint8_t out_value, uint8_t *in_value);

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
