/*
 * SHTC3.cpp
 *
 *  Created on: Mar. 30, 2022
 *      Author: malalonde
 */

#include "SHTC3.hpp"

#include <cmath>

#define SHTC3_ADDRESS 0x70
const uint8_t shtc3_wakeup[2] = { 0x35, 0x17 };
const uint8_t shtc3_sleep[2] = { 0xB0, 0x98 };
const uint8_t shtc3_meas[2] = { 0x78, 0x66 };
const uint8_t shtc3_detect[2] = { 0xEF, 0xC8 };
const uint8_t shtc3_reset[2] = { 0x80, 0x5D };

SHTC3::SHTC3(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t bus_speed, i2c_port_t i2c_num) {
	i2c_config_t config;
	config.mode = I2C_MODE_MASTER;
	config.sda_io_num = sda_io_num;
	config.scl_io_num = scl_io_num;
	config.sda_pullup_en = true;
	config.scl_pullup_en = true;
	config.master.clk_speed = bus_speed;
	config.clk_flags = 0x0000;

	m_i2c_driver = new I2CThreadSafeDriver(config, i2c_num);

}

SHTC3::~SHTC3() {
	delete m_i2c_driver;
}

esp_err_t SHTC3::update() {
	uint8_t data_bytes[6] = { 0 };
	esp_err_t err = ESP_OK;

	/* Wake up and wait for idle state (240 us) */
	err = m_i2c_driver->masterWrite(SHTC3_ADDRESS, shtc3_wakeup, 2);
	vTaskDelay(1);

	/* Request measurement and wait for it to be done (12.1 ms). */
	if (err == ESP_OK) {
		err = m_i2c_driver->masterWrite(SHTC3_ADDRESS, shtc3_meas, 2);
		vTaskDelay(pdMS_TO_TICKS(13) | 1);
	}
	if (err == ESP_OK) {
		err = m_i2c_driver->masterRead(SHTC3_ADDRESS, data_bytes, 6);
	}
	/* Put device back to sleep. */
	if (err == ESP_OK) {
		err = m_i2c_driver->masterWrite(SHTC3_ADDRESS, shtc3_sleep, 2);
	}

	// Validate data
	if (err == ESP_OK) {
		err = (SHTC3::computeCRC8(&(data_bytes[0]), 2) == data_bytes[2]) ? ESP_OK : ESP_ERR_INVALID_CRC;
	}
	if (err == ESP_OK) {
		err = (SHTC3::computeCRC8(&(data_bytes[3]), 2) == data_bytes[5]) ? ESP_OK : ESP_ERR_INVALID_CRC;
	}

	if (err == ESP_OK) {
		m_temperature_celsius = -45.0f + 175.0f * ((float) ((data_bytes[0] << 8) | data_bytes[1])) / 65536.0f;
		m_humidity_pct = 100.0f * ((float) ((data_bytes[3] << 8) | data_bytes[4])) / 65536.0f;
	} else {
		m_temperature_celsius = NAN;
		m_humidity_pct = NAN;
	}

	return err;
}

uint8_t SHTC3::computeCRC8(uint8_t data[], uint16_t len) {
	uint8_t crc = 0xff, j;
	uint16_t i;
	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			if ((crc & 0x80) != 0) {
				crc = (uint8_t)((crc << 1) ^ 0x31);
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

float SHTC3::getTemperature_celsius() {
	return m_temperature_celsius;
}

float SHTC3::getHumidity_pct() {
	return m_humidity_pct;
}
