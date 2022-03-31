/*
 * DS18B20_UART.cpp
 *
 *  Created on: Mar. 25, 2022
 *      Author: malalonde
 */

#include "DS18B20_UART.hpp"
#include <string.h>
#include <cmath>

#include "esp_log.h"

#define LOG_TAG "DS18B20_UART"

// DS18B20 ROM Commands

#define DS18B20_ALARM_SEARCH 0xec

// DS18B20 Function Commands
#define DS18B20_START_CONVERSION 0x44
#define DS18B20_READ_SCRATCHPAD 0xbe
#define DS18B20_WRITE_SCRATCHPAD 0x4e
#define DS18B20_COPY_SCRATCHPAD 0x48
#define DS18B20_RECALL_E2 0xb8
#define DS18B20_READ_POWER_SUPPLY 0xb4

DS18B20_UART::DS18B20_UART(gpio_num_t tx_io_num, gpio_num_t rx_io_num, std::array<uint8_t, 8> address,
		uint8_t resolution_bits, uart_port_t uart_num) :
		m_address(address) {
	assert(uart_num >= 0 && uart_num < UART_NUM_MAX);
	m_ow_driver = new OneWireUARTThreadSafeDriver(tx_io_num, rx_io_num, uart_num);
	assert(setResolution(resolution_bits) == ESP_OK);
	m_initialized = false;
	m_temperature_celsius = NAN;

}

DS18B20_UART::~DS18B20_UART() {
	delete m_ow_driver;
}

esp_err_t DS18B20_UART::initialize() {
	esp_err_t err = ESP_OK;
	// Configure resolution and alarms
	int8_t config[3] = { 40, -30, (int8_t)(((m_resolution_bits - 9) << 5) | 0x1F) };
	err = m_ow_driver->write(m_address, DS18B20_WRITE_SCRATCHPAD, (uint8_t*) config, 3);

	if (err == ESP_OK) {
		// Save configuration to EEPROM.
		err = m_ow_driver->write(m_address, DS18B20_COPY_SCRATCHPAD, NULL, 0);
		vTaskDelay(pdMS_TO_TICKS(10));	// As per Datasheet
	}
	if (err == ESP_OK) {
		m_initialized = true;
	}

	return err;
}

esp_err_t DS18B20_UART::startConversion() {
	if (m_initialized) {
		return m_ow_driver->write(m_address, DS18B20_START_CONVERSION, NULL, 0);
	} else {
		return ESP_FAIL;
	}

}

esp_err_t DS18B20_UART::startConversionAll() {
	if (m_initialized) {
		return m_ow_driver->writeAll(DS18B20_START_CONVERSION, NULL, 0);
	} else {
		return ESP_FAIL;
	}
}

esp_err_t DS18B20_UART::read() {
	if (m_initialized) {
		uint8_t read_buffer[9] = { 0 };
		esp_err_t err = m_ow_driver->read(m_address, DS18B20_READ_SCRATCHPAD, read_buffer, 9);

		ESP_LOGD(LOG_TAG, "Scratchpad: %02X %02X %02X %02X %02X %02X %02X %02X %02X", read_buffer[0],
				read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4], read_buffer[5],
				read_buffer[6], read_buffer[7], read_buffer[8]);

		if (err == ESP_OK) {
			err = OneWireUARTThreadSafeDriver::checkOneWireCRC(read_buffer, 9);
		}
		if (err == ESP_OK) {
			m_temperature_celsius = ((float) (read_buffer[0] | (read_buffer[1] << 8))) * 0.0625;
		} else {
			m_temperature_celsius = NAN;
		}

		return err;
	} else {
		return ESP_FAIL;
	}
}

esp_err_t DS18B20_UART::update() {
	esp_err_t err = this->startConversion();
	ESP_LOGD(LOG_TAG, "Conversion command result: %s", esp_err_to_name(err));
	if (err == ESP_OK) {
		vTaskDelay(pdMS_TO_TICKS(752 >> (12 - m_resolution_bits)));
		err = this->read();
	}

	return err;
}

float DS18B20_UART::getTemperature_celsius() {
	return m_temperature_celsius;
}

esp_err_t DS18B20_UART::setResolution(uint8_t resolution_bits) {
	esp_err_t err = ESP_OK;

	if (resolution_bits < 9 || resolution_bits > 12) {
		err = ESP_FAIL;
	} else {
		m_resolution_bits = resolution_bits;
	}
	return err;
}
