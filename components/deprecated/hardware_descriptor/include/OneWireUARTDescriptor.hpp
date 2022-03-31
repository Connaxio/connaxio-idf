/*
 * OneWireDescriptor.h
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_HARDWAREDESCRIPTOR_ONEWIREDESCRIPTOR_HPP_
#define COMPONENTS_HARDWAREDESCRIPTOR_ONEWIREDESCRIPTOR_HPP_

#include "HardwareDescriptor.hpp"

#include <stdint.h>
#include "driver/gpio.h"

#include "esp_err.h"

class OneWireUARTDescriptor: public HardwareDescriptor {
public:
	OneWireUARTDescriptor(gpio_num_t t_tx_gpio_num, gpio_num_t t_rx_gpio_num);
	~OneWireUARTDescriptor();

	/* Search for a device of the specified family ID on the bus. */
	esp_err_t findDevice(uint8_t family_id);

	/*! \brief  Sends one byte of data on the 1-Wire(R) bus(es).
	 *
	 *  This function automates the task of sending a complete byte
	 *  of data on the 1-Wire bus(es).
	 *
	 *  \param  data    The data to send on the bus(es).
	 *  \param  pins    A bitmask of the buses to send the data to.
	 */
	void writeByte(uint8_t data);

	/*! \brief  Receives one byte of data from the 1-Wire(R) bus.
	 *
	 *  This function automates the task of receiving a complete byte
	 *  of data from the 1-Wire bus.
	 *
	 *  \param  pin     A bitmask of the bus to read from.
	 *  \return     The byte read from the bus.
	 */
	uint8_t readByte();

	/*! \brief  Sends the SKIP ROM command to the 1-Wire bus(es).
	 *
	 *  \param  pins    A bitmask of the buses to send the SKIP ROM command to.
	 */
	void skipROM();

	/*! \brief  Sends the READ ROM command and reads back the ROM id.
	 *
	 *  \param  romValue    A pointer where the id will be placed.
	 *  \param  pin     A bitmask of the bus to read from.
	 */
	void readROM();

	/*! \brief  Sends the MATCH ROM command and the ROM id to match against.
	 *
	 *  \param  romValue    A pointer to the ID to match against.
	 *  \param  pins    A bitmask of the buses to perform the MATCH ROM command on.
	 */
	esp_err_t matchROM();

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
	esp_err_t searchROM(unsigned char lastDeviation);

	/*! \brief  Read a bit from the bus. (Polled UART DRIVER)
	 *
	 *  Generates the waveform for reception of a bit on the 1-Wire(R) bus(es).
	 *
	 *  \return The value read from the bus (0 or 1).
	 */
	uint8_t readBit();

	bool hasValidROM();

	void resetROM();

	static uint8_t getUARTNum();

private:
	gpio_num_t m_tx_gpio_num;
	gpio_num_t m_rx_gpio_num;
	uint8_t m_device_id[8];

	static uint8_t m_uart_num;
	static const uint8_t m_uart_buffer_size = SOC_UART_FIFO_LEN + 8;
	static QueueHandle_t m_uart_queue_handle;

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
	static esp_err_t checkROMCRC(unsigned char *romValue);

	/*! \brief  Write and read one bit to/from the 1-Wire bus. (Polled UART driver)
	 *
	 *  Writes one bit to the bus and returns the value read from the bus.
	 *
	 *  \param  outValue    The value to transmit on the bus.
	 *
	 *  \return The value received by the UART from the bus.
	 */
	uint8_t touchBit(unsigned char outValue);

	/*! \brief Write a '1' bit to the bus. (Polled UART DRIVER)
	 *
	 *  Generates the waveform for transmission of a '1' bit on the 1-Wire
	 *  bus.
	 */
	void write1();

	/*! \brief  Write a '0' to the bus. (Polled UART DRIVER)
	 *
	 *  Generates the waveform for transmission of a '0' bit on the 1-Wire(R)
	 *  bus.
	 */
	void write0();

	/*! \brief  Send a Reset signal and listen for Presence signal. (Polled
	 *  UART DRIVER)
	 *
	 *  Generates the waveform for transmission of a Reset pulse on the
	 *  1-Wire(R) bus and listens for presence signals.
	 *
	 *  \return non zero value when a presence signal was detected.
	 */
	esp_err_t detectPresence();

};

#endif /* COMPONENTS_HARDWAREDESCRIPTOR_ONEWIREDESCRIPTOR_HPP_ */
