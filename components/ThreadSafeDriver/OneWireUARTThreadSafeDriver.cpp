/*
 * OneWireUARTThreadSafeDriver.cpp
 *
 *  Created on: Mar. 23, 2022
 *      Author: malalonde
 */

#include "OneWireUARTThreadSafeDriver.hpp"

#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"

#define LOG_TAG "OneWireUARTThreadSafeDriver"

/****************************************************************************
 ROM commands
 ****************************************************************************/
#define OW_ROM_READ 0x33   //!< READ ROM command code.
#define OW_ROM_SKIP 0xcc   //!< SKIP ROM command code.
#define OW_ROM_MATCH 0x55  //!< MATCH ROM command code.
#define OW_ROM_SEARCH 0xf0 //!< SEARCH ROM command code.

/****************************************************************************
 Return codes
 ****************************************************************************/
#define OW_ROM_SEARCH_FINISHED 0x00 //!< Search finished return code.
#define OW_ROM_SEARCH_FAILED 0xff   //!< Search failed return code.

/****************************************************************************
 UART patterns
 ****************************************************************************/
#define OW_UART_WRITE1 0xff   //!< UART Write 1 bit pattern.
#define OW_UART_WRITE0 0x00   //!< UART Write 0 bit pattern.
#define OW_UART_READ_BIT 0xff //!< UART Read bit pattern.
#define OW_UART_RESET 0xf0    //!< UART Reset bit pattern.

OneWireUARTThreadSafeDriver::OneWireUARTThreadSafeDriver(uart_port_t uart_num, gpio_num_t tx_io_num,
		gpio_num_t rx_io_num) {
	uart_config_t uart_config = { // @suppress("Invalid arguments")
			.baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits =
					UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .rx_flow_ctrl_thresh = 122,
					.source_clk = UART_SCLK_APB };

	m_uart_driver = new UARTThreadSafeDriver(uart_config, uart_num, tx_io_num, rx_io_num);
	m_num_devices = 0;

}

OneWireUARTThreadSafeDriver::~OneWireUARTThreadSafeDriver() {
	delete (m_uart_driver);
	free(m_addresses);
}

uint8_t OneWireUARTThreadSafeDriver::computeCRC8(unsigned char data_in, unsigned char seed) {
	unsigned char bitsLeft;
	unsigned char temp;

	for (bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
		temp = ((seed ^ data_in) & 0x01);
		if (temp == 0) {
			seed >>= 1;
		} else {
			seed ^= 0x18;
			seed >>= 1;
			seed |= 0x80;
		}
		data_in >>= 1;
	}
	return seed;
}

uint32_t OneWireUARTThreadSafeDriver::computeCRC16(unsigned char data_in, unsigned int seed) {
	uint8_t temp;

	for (uint8_t i = 0; i < 8; ++i) {
		temp = ((seed ^ data_in) & 0x01);
		if (temp == 0) {
			seed >>= 1;
		} else {
			seed ^= 0x4002;
			seed >>= 1;
			seed |= 0x8000;
		}
		data_in >>= 1;
	}
	return seed;
}

esp_err_t OneWireUARTThreadSafeDriver::checkROMCRC(uint8_t (&address)[8]) {
	uint8_t crc8 = 0;

	for (uint8_t i = 0; i < 7; i++) {
		crc8 = OneWireUARTThreadSafeDriver::computeCRC8(address[i], crc8);
	}

	return crc8 == (address[7]) ? ESP_OK : ESP_FAIL;
}

int8_t OneWireUARTThreadSafeDriver::findFamilyDevice(uint8_t family_id, uint8_t search_begin_index,
		uint8_t (&address)[8]) {
	memset(address, 0, 8);
	int8_t idx_found = -1;

	for (uint8_t i = search_begin_index; i < m_num_devices && idx_found == -1; ++i) {
		if (m_addresses[i][0] == family_id) {
			memcpy(address, m_addresses[i], 8);
			idx_found = i;
		}
	}

	return idx_found;
}

esp_err_t OneWireUARTThreadSafeDriver::writeByte(uint8_t data) {
	esp_err_t err = ESP_OK;

	// Do once for each bit
	for (uint8_t i = 0; i < 8 && err == ESP_OK; i++) {
		// Determine if lsb is '0' or '1' and transmit corresponding
		// waveform on the bus.
		if (data & 0x01) {
			err = this->write1();
		} else {
			err = this->write0();
		}
		// Right shift the data to get next bit.
		data >>= 1;
	}
	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::readByte(uint8_t &data) {
	esp_err_t err = ESP_OK;
	data = 0;
	uint8_t bit = 0;

	// Do once for each bit
	for (uint8_t i = 0; i < 8 && err == ESP_OK; i++) {
		// Shift input variable right.
		data >>= 1;
		// Set the msb if a '1' value is read from the bus.
		// Leave as it is ('0') else.
		err = this->readBit(bit);
		if (err == ESP_OK && bit) {
			// Set msb
			data |= 0x80;
		}
	}
	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::skipROM() {
	// Send the SKIP ROM command on the bus.
	return this->writeByte(OW_ROM_SKIP);
}

esp_err_t OneWireUARTThreadSafeDriver::readROM(uint8_t (&address)[8]) {
	// Send the READ ROM command on the bus.
	esp_err_t err = this->writeByte(OW_ROM_READ);

	// Do 8 times.
	for (uint8_t i = 0; i < 8 && err == ESP_OK; ++i) {
		// Place the received data in memory.
		err = this->readByte(address[i]);
	}

	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::matchROM(uint8_t (&address)[8]) {
	esp_err_t err = ESP_OK;
	uint8_t presence = 0;
	// Reset, presence.
	err = this->detectPresence(presence);

	// Send the MATCH ROM command.
	if (err == ESP_OK && presence) {
		err = this->writeByte(OW_ROM_MATCH);
	}

	// Do once for each byte.
	for (uint8_t i = 0; i < 8 && err == ESP_OK && presence; ++i) {
		// Transmit 1 byte of the ID to match.
		err = this->writeByte(address[i]);
	}

	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::searchROM(uint8_t &lastDeviation, uint8_t (&address)[8]) {
	unsigned char newDeviation = 0;
	unsigned char bitMask = 0x01;
	unsigned char bitA;
	unsigned char bitB;
	uint8_t byteIndex = 0;
	esp_err_t err = ESP_OK;

// Send SEARCH ROM command on the bus.
	err = this->writeByte(OW_ROM_SEARCH);

// Walk through all 64 bits.

	for (uint8_t currentBit = 1; currentBit <= 64 && err == ESP_OK; ++currentBit) {
		// Read bit from bus twice.
		err = this->readBit(bitA);
		if (err == ESP_OK) {
			err = this->readBit(bitB);
		}

		if (bitA && bitB) {
			// Both bits 1 (Error).
			err = ESP_FAIL;
		} else if (bitA ^ bitB) {
			// Bits A and B are different. All devices have the same bit here.
			// Set the bit in bitPattern to this value.
			if (bitA) {
				(address[byteIndex]) |= bitMask;
			} else {
				(address[byteIndex]) &= ~bitMask;
			}
		} else // Both bits 0
		{
			// If this is where a choice was made the last time,
			// a '1' bit is selected this time.
			if (currentBit == lastDeviation) {
				(address[byteIndex]) |= bitMask;
			}
			// For the rest of the id, '0' bits are selected when
			// discrepancies occur.
			else if (currentBit > lastDeviation) {
				(address[byteIndex]) &= ~bitMask;
				newDeviation = currentBit;
			}
			// If current bit in bit pattern = 0, then this is
			// out new deviation.
			else if (!(address[byteIndex] & bitMask)) {
				newDeviation = currentBit;
			}
			// IF the bit is already 1, do nothing.
			else {
			}
		}

		// Send the selected bit to the bus.
		if ((address[byteIndex]) & bitMask && err == ESP_OK) {
			err = this->write1();
		} else {
			err = this->write0();
		}

		// Adjust bitMask and bitPattern pointer.
		bitMask <<= 1;
		if (!bitMask) {
			bitMask = 0x01;
			byteIndex++;
		}
	}
	lastDeviation = newDeviation;

	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::searchBus() {
	uint8_t address_tmp[8] = { 0 };
	uint8_t last_deviation = 0;
	esp_err_t err = ESP_OK;
	uint8_t presence = 0;

	// Find the buses with slave devices.
	err = this->detectPresence(presence);

	// Go through all buses with slave devices.
	if (err == ESP_OK && presence) // Devices available on this bus.
			{
		// Do slave search on each bus, and place identifiers and corresponding
		// bus "addresses" in the array.
		do {
			err = this->detectPresence(presence);
			if (err == ESP_OK && presence) {
				// Find a new device by searching from previous address
				err = this->searchROM(last_deviation, address_tmp);
			}
			if (err == ESP_OK && presence) {
				// Add new device to list
				m_addresses = (uint8_t (*)[8]) realloc(m_addresses, sizeof(uint8_t[m_num_devices + 1][8]));
				memcpy(m_addresses[m_num_devices], address_tmp, 8);
				m_num_devices++;
			}

		} while (last_deviation != OW_ROM_SEARCH_FINISHED);
	}

	// Go through all the devices and do CRC check.
	for (uint8_t i = 0; i < m_num_devices && err == ESP_OK; i++) {
		err = OneWireUARTThreadSafeDriver::checkROMCRC(m_addresses[i]);
	}
	// Else, return Successful.
	return err;
}

bool OneWireUARTThreadSafeDriver::hasValidROM(uint8_t (&address)[8]) {
	return (OneWireUARTThreadSafeDriver::checkROMCRC(address) == ESP_OK) && (uint64_t)(address[0]) != 0;
}

esp_err_t OneWireUARTThreadSafeDriver::touchBit(uint8_t output_value, uint8_t *input_value) {
	return m_uart_driver->poll(&output_value, 1, input_value, 1);
}

esp_err_t OneWireUARTThreadSafeDriver::readBit(uint8_t &bit) {
// Return 1 if the value received matches the value sent.
// Return 0 else. (A slave held the bus low).
	uint8_t bit_tmp;
	esp_err_t err = this->touchBit(OW_UART_READ_BIT, &bit_tmp);
	bit = (bit_tmp == OW_UART_READ_BIT);
	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::write1() {
	uint8_t bit = OW_UART_WRITE1;
	return m_uart_driver->write(&bit, 1);
}

esp_err_t OneWireUARTThreadSafeDriver::write0() {
	uint8_t bit = OW_UART_WRITE0;
	return m_uart_driver->write(&bit, 1);
}

esp_err_t OneWireUARTThreadSafeDriver::detectPresence(uint8_t &presence) {
	esp_err_t err = ESP_OK;
	uint8_t bit;
	err = m_uart_driver->setBaudrate(9600);
	if (err == ESP_OK) {
		err = this->touchBit(OW_UART_RESET, &bit);
	}
	if (err == ESP_OK) {
		presence = (bit == OW_UART_RESET);
	}
	if (err == ESP_OK) {
		err = m_uart_driver->setBaudrate(115200);
	}

	ESP_LOGD(LOG_TAG, "Detect presence: %s. Found: %i.", esp_err_to_name(err), presence);
	return err;
}

uint8_t OneWireUARTThreadSafeDriver::getUARTNum() {
	return m_uart_driver->getPort();
}

