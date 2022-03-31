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

OneWireUARTThreadSafeDriver::OneWireUARTThreadSafeDriver(gpio_num_t tx_io_num, gpio_num_t rx_io_num,
		uart_port_t uart_num) {
	uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 122,
			.source_clk = UART_SCLK_APB };

	m_uart_driver = new UARTThreadSafeDriver(uart_config, uart_num, tx_io_num, rx_io_num);
	m_uart_driver->setTimeout_ms(100);
}

OneWireUARTThreadSafeDriver::~OneWireUARTThreadSafeDriver() {
	delete m_uart_driver;
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

esp_err_t OneWireUARTThreadSafeDriver::checkOneWireCRC(uint8_t data[], uint8_t length) {
	uint8_t crc8 = 0;

	for (uint8_t i = 0; i < length - 1; i++) {
		crc8 = OneWireUARTThreadSafeDriver::computeCRC8(data[i], crc8);
	}

	return crc8 == (data[length - 1]) ? ESP_OK : ESP_FAIL;
}

std::array<uint8_t, 8> OneWireUARTThreadSafeDriver::findFamilyDevice(uint8_t family_id,
		uint8_t search_begin_index, std::vector<std::array<uint8_t, 8>> addresses) {
	std::array < uint8_t, 8 > address = { 0, 0, 0, 0, 0, 0, 0, 0 };

	for (uint8_t i = search_begin_index; i < addresses.size(); ++i) {
		if (addresses.at(i)[0] == family_id) {
			address = addresses.at(i);
			break;
		}
	}

	return address;
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

esp_err_t OneWireUARTThreadSafeDriver::write(std::array<uint8_t, 8> address, uint8_t code, uint8_t data[],
		uint8_t length) {
	uint8_t uart_data[80 + 8 * length] = { 0 };
	// Add MATCH_ROM command
	for (uint8_t i = 0; i < 8; ++i) {
		uart_data[i] = ((OW_ROM_MATCH >> i) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
	}
	// Add ROM / ID / ADDRESS
	for (uint8_t i = 0; i < 8; ++i) {
		for (uint8_t j = 0; j < 8; ++j) {
			uart_data[8 * i + j + 8] = ((address[i] >> j) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
		}
	}
	// Add command code
	for (uint8_t i = 0; i < 8; ++i) {
		uart_data[i + 72] = ((code >> i) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
	}
	// Add command data
	for (uint8_t i = 0; i < length; ++i) {
		for (uint8_t j = 0; j < 8; ++j) {
			uart_data[8 * i + j + 80] = ((data[i] >> j) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
		}
	}
	// Write data
	uint8_t presence = 0;
	esp_err_t err = this->detectPresence(presence);
	if (err == ESP_OK && presence) {
		err = m_uart_driver->write(uart_data, 80 + 8 * length);
	}
	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::writeAll(uint8_t code, uint8_t data[], uint8_t length) {
	// Prepare data
	uint8_t uart_data[16 + 8 * length] = { 0 };
	// Add SKIP_ROM command
	for (uint8_t i = 0; i < 8; ++i) {
		uart_data[i] = ((OW_ROM_SKIP >> i) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
	}
	// Add command code
	for (uint8_t i = 0; i < 8; ++i) {
		uart_data[i + 8] = ((code >> i) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
	}
	// Add command data
	for (uint8_t i = 0; i < length; ++i) {
		for (uint8_t j = 0; j < 8; ++j) {
			uart_data[8 * i + j + 16] = ((data[i] >> j) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
		}
	}
	// Write data
	uint8_t presence = 0;
	esp_err_t err = this->detectPresence(presence);
	if (err == ESP_OK && presence) {
		err = m_uart_driver->write(uart_data, 16 + 8 * length);
	}
	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::read(std::array<uint8_t, 8> address, uint8_t code, uint8_t buffer[],
		uint8_t length) {
	uint8_t uart_write_data[80 + 8 * length] = { 0 };
	uint8_t uart_read_data[80 + 8 * length] = { 0 };
	// Add MATCH_ROM command
	for (uint8_t i = 0; i < 8; ++i) {
		uart_write_data[i] = ((OW_ROM_MATCH >> i) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
	}
	for (uint8_t i = 0; i < 8; ++i) {
		for (uint8_t j = 0; j < 8; ++j) {
			uart_write_data[8 * i + j + 8] = ((address[i] >> j) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
		}
	}
	// Add command code
	for (uint8_t i = 0; i < 8; ++i) {
		uart_write_data[i + 72] = ((code >> i) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
	}
	// Add read bytes
	for (uint8_t i = 0; i < 8 * length; ++i) {
		uart_write_data[i + 80] = OW_UART_READ_BIT;
	}
	// Write/read data
	uint8_t presence = 0;
	esp_err_t err = this->detectPresence(presence);
	if (err == ESP_OK && presence) {
		err = m_uart_driver->poll(uart_write_data, 80 + 8 * length, uart_read_data, 80 + 8 * length);
	}

	// Extract data
	for (uint8_t i = 0; i < length; ++i) {
		buffer[i] = 0;
		for (uint8_t j = 0; j < 8; ++j) {
			buffer[i] |= (uart_read_data[8 * i + j + 80] == OW_UART_READ_BIT) << j;
		}
	}

	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::readAny(uint8_t code, uint8_t buffer[], uint8_t length) {
	uint8_t uart_write_data[8 + 8 * length] = { 0 };
	uint8_t uart_read_data[8 + 8 * length] = { 0 };
	// Add command code
	for (uint8_t i = 0; i < 8; ++i) {
		uart_write_data[i] = ((code >> i) & 0x01) ? OW_UART_WRITE1 : OW_UART_WRITE0;
	}
	// Add read bytes
	for (uint8_t i = 0; i < 8 * length; ++i) {
		uart_write_data[i + 8] = OW_UART_READ_BIT;
	}

	// Write/read data
	uint8_t presence = 0;
	esp_err_t err = this->detectPresence(presence);
	if (err == ESP_OK && presence) {
		err = m_uart_driver->poll(uart_write_data, 80 + 8 * length, uart_read_data, 80 + 8 * length);
	}

	// Extract data
	for (uint8_t i = 0; i < length; ++i) {
		buffer[i] = 0;
		for (uint8_t j = 0; j < 8; ++j) {
			buffer[i] |= (uart_read_data[8 * i + j + 80] == OW_UART_READ_BIT) << j;
		}
	}
	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::readROM(std::array<uint8_t, 8> address) {
// Send the READ ROM command on the bus.
	return readAny(OW_ROM_READ, address.data(), 8);
}

esp_err_t OneWireUARTThreadSafeDriver::searchROM(uint8_t &lastDeviation, std::array<uint8_t, 8> &address) {
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

	ESP_LOGI(LOG_TAG, "Found device:\t%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", address[0], address[1],
			address[2], address[3], address[4], address[5], address[6], address[7]);

	return err;
}

esp_err_t OneWireUARTThreadSafeDriver::searchBus(std::vector<std::array<uint8_t, 8>> &addresses) {
	std::array < uint8_t, 8 > address_tmp = { 0, 0, 0, 0, 0, 0, 0, 0 };
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
			if (err == ESP_OK && presence
					&& OneWireUARTThreadSafeDriver::checkOneWireCRC(address_tmp.data(), 8) == ESP_OK) {
				ESP_LOGD(LOG_TAG, "Found device family:\t0x%02X", address_tmp[0]);
				// New address is valid
				addresses.push_back(address_tmp);
			}

		} while (last_deviation != OW_ROM_SEARCH_FINISHED && err == ESP_OK);
	}

	return err;
}

bool OneWireUARTThreadSafeDriver::hasValidROM(std::array<uint8_t, 8> address) {
	return (OneWireUARTThreadSafeDriver::checkOneWireCRC(address.data(), 8) == ESP_OK)
			&& (uint64_t)(address.data()) != 0;
}

esp_err_t OneWireUARTThreadSafeDriver::readBit(uint8_t &bit) {
// Return 1 if the value received matches the value sent.
// Return 0 else. (A slave held the bus low).
	uint8_t bit_read = OW_UART_READ_BIT;
	uint8_t bit_tmp;
	esp_err_t err = m_uart_driver->poll(&bit_read, 1, &bit_tmp, 1);
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
	m_uart_driver->setBaudrate(9600);
	if (err == ESP_OK) {
		uint8_t reset_bit = OW_UART_RESET;
		err = m_uart_driver->poll(&reset_bit, 1, &bit, 1);
	}
	if (err == ESP_OK) {
		presence = (bit != OW_UART_RESET);
	}

	m_uart_driver->setBaudrate(115200);

	ESP_LOGD(LOG_TAG, "Detect presence: %s. Found: %i.", esp_err_to_name(err), presence);
	return err;
}

uint8_t OneWireUARTThreadSafeDriver::getUARTNum() {
	return m_uart_driver->getPort();
}

