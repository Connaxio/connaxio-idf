/*
 * OneWireDescriptor.cpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#include "OneWireUARTDescriptor.hpp"
#include "driver/uart.h"
#include "esp_log.h"

#define LOG_TAG "OneWireUARTDescriptor"

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

uint8_t OneWireUARTDescriptor::m_uart_num = UART_NUM_MAX - 1;
QueueHandle_t OneWireUARTDescriptor::m_uart_queue_handle;

OneWireUARTDescriptor::OneWireUARTDescriptor(gpio_num_t t_tx_gpio_num, gpio_num_t t_rx_gpio_num) :
		m_tx_gpio_num(t_tx_gpio_num), m_rx_gpio_num(t_rx_gpio_num) {
	uart_config_t t_uart_config = { // @suppress("Invalid arguments")
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity =
			UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 122,
			.source_clk = UART_SCLK_APB };

	esp_err_t err = uart_param_config(m_uart_num, &t_uart_config);
	if (err == ESP_OK) {
		err = uart_driver_install(m_uart_num, OneWireUARTDescriptor::m_uart_buffer_size,
				OneWireUARTDescriptor::m_uart_buffer_size, 10, &OneWireUARTDescriptor::m_uart_queue_handle,
				0);
	}

	assert(err == ESP_OK);
}

OneWireUARTDescriptor::~OneWireUARTDescriptor() {
	// TODO Auto-generated destructor stub
}

uint8_t OneWireUARTDescriptor::computeCRC8(unsigned char data_in, unsigned char seed) {
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

uint32_t OneWireUARTDescriptor::computeCRC16(unsigned char data_in, unsigned int seed) {
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

esp_err_t OneWireUARTDescriptor::checkROMCRC(unsigned char *romValue) {
	uint8_t crc8 = 0;

	for (uint8_t i = 0; i < 7; i++) {
		crc8 = computeCRC8(*romValue, crc8);
		romValue++;
	}
	if (crc8 == (*romValue)) {
		return ESP_OK;
	}

	return ESP_FAIL;
}

esp_err_t OneWireUARTDescriptor::findDevice(uint8_t family_id) {
	esp_err_t device_found = ESP_FAIL;

	/* Initialize address */
	for (uint8_t i = 0; i < 8; i++) {
		m_device_id[i] = 0;
	}

	//detectPresence();	// Send a first reset.
	if (this->detectPresence() == ESP_OK) {
		this->searchROM(family_id);

		if (checkROMCRC(m_device_id) == ESP_OK) {
			device_found = ESP_OK;
			ESP_LOGI(LOG_TAG, "Found device ID: 0x%04X%04X", m_device_id[0], m_device_id[4]);
		} else {
			ESP_LOGI(LOG_TAG, "Invalid CRC found.");
		}

	} else {
		ESP_LOGI(LOG_TAG, "No device detected.");
	}


	return device_found;
}

void OneWireUARTDescriptor::writeByte(uint8_t data) {
	unsigned char temp;
	unsigned char i;

	// Do once for each bit
	for (i = 0; i < 8; i++) {
		// Determine if lsb is '0' or '1' and transmit corresponding
		// waveform on the bus.
		temp = data & 0x01;
		if (temp) {
			this->write1();
		} else {
			this->write0();
		}
		// Right shift the data to get next bit.
		data >>= 1;
	}
}

uint8_t OneWireUARTDescriptor::readByte() {
	unsigned char data;
	unsigned char i;

	// Clear the temporary input variable.
	data = 0x00;

	// Do once for each bit
	for (i = 0; i < 8; i++) {
		// Shift temporary input variable right.
		data >>= 1;
		// Set the msb if a '1' value is read from the bus.
		// Leave as it is ('0') else.
		if (this->readBit()) {
			// Set msb
			data |= 0x80;
		}
	}
	return data;
}

void OneWireUARTDescriptor::skipROM() {
	// Send the SKIP ROM command on the bus.
	this->writeByte(OW_ROM_SKIP);
}

void OneWireUARTDescriptor::readROM() {
	// Send the READ ROM command on the bus.
	this->writeByte(OW_ROM_READ);

	// Do 8 times.
	for (uint8_t i = 0; i < 8; ++i) {
		// Place the received data in memory.
		m_device_id[i] = this->readByte();
	}
}

esp_err_t OneWireUARTDescriptor::matchROM() {

// Reset, presence.
	if (this->detectPresence() != ESP_OK) {
		return ESP_FAIL; // Error
	}

// Send the MATCH ROM command.
	this->writeByte(OW_ROM_MATCH);

// Do once for each byte.
	for (uint8_t i = 0; i < 8; ++i) {
		// Transmit 1 byte of the ID to match.
		this->writeByte(m_device_id[i]);
	}

	return ESP_OK;
}

esp_err_t OneWireUARTDescriptor::searchROM(unsigned char lastDeviation) {
	unsigned char newDeviation = 0;
	unsigned char bitMask = 0x01;
	unsigned char bitA;
	unsigned char bitB;
	uint8_t byteIndex = 0;

// Send SEARCH ROM command on the bus.
	this->writeByte(OW_ROM_SEARCH);

// Walk through all 64 bits.

	for (uint8_t currentBit = 1; currentBit <= 64; ++currentBit) {
		// Read bit from bus twice.
		bitA = this->readBit();
		bitB = this->readBit();

		if (bitA && bitB) {
			// Both bits 1 (Error).
			return ESP_FAIL;
		} else if (bitA ^ bitB) {
			// Bits A and B are different. All devices have the same bit here.
			// Set the bit in bitPattern to this value.
			if (bitA) {
				(m_device_id[byteIndex]) |= bitMask;
			} else {
				(m_device_id[byteIndex]) &= ~bitMask;
			}
		} else // Both bits 0
		{
			// If this is where a choice was made the last time,
			// a '1' bit is selected this time.
			if (currentBit == lastDeviation) {
				(m_device_id[byteIndex]) |= bitMask;
			}
			// For the rest of the id, '0' bits are selected when
			// discrepancies occur.
			else if (currentBit > lastDeviation) {
				(m_device_id[byteIndex]) &= ~bitMask;
				newDeviation = currentBit;
			}
			// If current bit in bit pattern = 0, then this is
			// out new deviation.
			else if (!(m_device_id[byteIndex] & bitMask)) {
				newDeviation = currentBit;
			}
			// IF the bit is already 1, do nothing.
			else {
			}
		}

		// Send the selected bit to the bus.
		if ((m_device_id[byteIndex]) & bitMask) {
			this->write1();
		} else {
			this->write0();
		}

		// Adjust bitMask and bitPattern pointer.
		bitMask <<= 1;
		if (!bitMask) {
			bitMask = 0x01;
			byteIndex++;
		}
	}

	if (newDeviation == OW_ROM_SEARCH_FINISHED) {
		return ESP_OK;
	} else {
		return ESP_FAIL;
	}
}

bool OneWireUARTDescriptor::hasValidROM() {
	return (OneWireUARTDescriptor::checkROMCRC(m_device_id) == ESP_OK) && (uint64_t) (m_device_id[0]) != 0;
}

void OneWireUARTDescriptor::resetROM(){
	for(uint8_t i = 0; i < 8; ++i){
		m_device_id[i] = 0;
	}
}

uint8_t OneWireUARTDescriptor::touchBit(unsigned char outValue) {
	uint8_t inValue = outValue;

	// Set pins for required port
	ESP_ERROR_CHECK(
			uart_set_pin(m_uart_num, m_tx_gpio_num, m_rx_gpio_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	// Prepare input
	ESP_ERROR_CHECK(uart_flush_input(m_uart_num));
	// Write bit to OneWire bus
	uart_write_bytes(m_uart_num, &outValue, 1);

	//Wait for reception
	size_t length = 0;
	for(uint8_t i = 0; i < 20 && length == 0; ++i){
		vTaskDelay(1);
		ESP_ERROR_CHECK(uart_get_buffered_data_len(m_uart_num, (size_t* )&length));
	}
	// Read answer
	if (length > 0) {
		length = uart_read_bytes(m_uart_num, &inValue, 1, pdMS_TO_TICKS(10));
	}

	return inValue;
}

uint8_t OneWireUARTDescriptor::readBit() {
// Return 1 if the value received matches the value sent.
// Return 0 else. (A slave held the bus low).
	return (this->touchBit(OW_UART_READ_BIT) == OW_UART_READ_BIT);
}

void OneWireUARTDescriptor::write1() {
	this->touchBit(OW_UART_WRITE1);
}

void OneWireUARTDescriptor::write0() {
	this->touchBit(OW_UART_WRITE0);
}

esp_err_t OneWireUARTDescriptor::detectPresence() {
	ESP_ERROR_CHECK(uart_set_baudrate(m_uart_num, 9600));
	esp_err_t err = (this->touchBit(OW_UART_RESET) != OW_UART_RESET) ? ESP_OK : ESP_FAIL;
	ESP_ERROR_CHECK(uart_set_baudrate(m_uart_num, 115200));

	ESP_LOGD(LOG_TAG, "Detect presence: %s", esp_err_to_name(err));
	return err;
}

uint8_t OneWireUARTDescriptor::getUARTNum() {
	return m_uart_num;
}
