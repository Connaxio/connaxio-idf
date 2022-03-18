/*
 * SHTC3.cpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#include "SHTC3.hpp"

#include <math.h>
#include "I2CDescriptor.hpp"

#define SHTC3_I2C_NUM 	    I2C_NUM_0
#define TOPIC_TEMPERATURE   "temperature"
#define TOPIC_HUMIDITY      "humidity"

const uint8_t SHTC3::m_cmd_wakeup[2] = { 0x35, 0x17 };
const uint8_t SHTC3::m_cmd_sleep[2] = { 0xB0, 0x98 };
const uint8_t SHTC3::m_cmd_meas[2] = { 0x78, 0x66 };
const uint8_t SHTC3::m_cmd_detect[2] = { 0xEF, 0xC8 };
const uint8_t SHTC3::m_cmd_reset[2] = { 0x80, 0x5D };


SHTC3::SHTC3(std::string t_name, gpio_num_t t_sda_gpio_num, gpio_num_t t_scl_gpio_num,
		gpio_num_t t_enable_pin, uint16_t t_turn_on_time_ms, uint8_t t_enable_level) :
		Peripheral(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level),
		Sensor(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level) {
	m_values[TOPIC_TEMPERATURE] = NAN;
	m_values[TOPIC_HUMIDITY] = NAN;
	m_hardware_descriptor = new I2CDescriptor(t_sda_gpio_num, t_scl_gpio_num, 1000000, SHTC3_I2C_NUM,
			m_address);

}

SHTC3::~SHTC3() {
	delete (m_hardware_descriptor);
}

uint8_t SHTC3::computeCRC8(uint8_t data[], uint16_t len) {
	uint8_t crc = 0xff, j;
	uint16_t i;
	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			if ((crc & 0x80) != 0)
				crc = (uint8_t) ((crc << 1) ^ 0x31);
			else
				crc <<= 1;
		}
	}
	return crc;
}

esp_err_t SHTC3::update() {
	uint8_t data_bytes[6] = { 0 };
	esp_err_t err = ESP_OK, err_temp = ESP_ERR_INVALID_CRC, err_humi = ESP_ERR_INVALID_CRC;
	uint16_t temp, humi;
	I2CDescriptor *i2c_desc = static_cast<I2CDescriptor*>(m_hardware_descriptor);

	HardwareDescriptor::lockI2C(i2c_desc->getPort());
	i2c_desc->init();

	/* Wake up and wait for idle state (240 us) */
	err = i2c_desc->write(SHTC3::m_cmd_wakeup, 2);
	vTaskDelay( 1 );

	for (uint8_t i = 0; i < 10 && err == ESP_OK && (err_temp != ESP_OK || err_humi != ESP_OK); ++i) {

		/* Sometimes the device takes longer to wake up (hardware bug?), try to read in loop. */
		/* Request measurement and wait for it to be done (12.1 ms). */
		if (err == ESP_OK) {
			err = i2c_desc->write(SHTC3::m_cmd_meas, 2);
			vTaskDelay( pdMS_TO_TICKS( 13 ) + 1);
		}
		if (err == ESP_OK) {
			err = i2c_desc->read(data_bytes, 6);
		}

		// Validate data
		err_temp = (SHTC3::computeCRC8(data_bytes + 0, 2) == data_bytes[2]) ? ESP_OK : ESP_ERR_INVALID_CRC;
		err_humi = (SHTC3::computeCRC8(data_bytes + 3, 2) == data_bytes[5]) ? ESP_OK : ESP_ERR_INVALID_CRC;
	}

	/* Put device back to sleep. */
	if (err == ESP_OK) {
		err = i2c_desc->write(SHTC3::m_cmd_sleep, 2);
	}

	i2c_desc->deinit();
	HardwareDescriptor::unlockI2C(i2c_desc->getPort());

	if (err_temp == ESP_OK) {
		temp = (data_bytes[0] << 8) | data_bytes[1];
		m_values[TOPIC_TEMPERATURE] = -45.0f + 175.0f * ((float) (temp)) / 65536.0f;
	} else {
		m_values[TOPIC_TEMPERATURE] = NAN;
	}

	if (err_humi == ESP_OK) {
		humi = (data_bytes[3] << 8) | data_bytes[4];
		m_values[TOPIC_HUMIDITY] = 100.0f * ((float) (humi)) / 65536.0f;
	} else {
		m_values[TOPIC_HUMIDITY] = NAN;
	}

	return (err_temp == ESP_OK && err_humi == ESP_OK) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

esp_err_t SHTC3::init() {
	esp_err_t err = ESP_OK;

	I2CDescriptor *i2c_desc = static_cast<I2CDescriptor*>(m_hardware_descriptor);

	HardwareDescriptor::lockI2C(i2c_desc->getPort());
	i2c_desc->init();

	err = i2c_desc->write(SHTC3::m_cmd_wakeup, 2);
	vTaskDelay( 1 );

	if (err == ESP_OK) {
		err = i2c_desc->write(SHTC3::m_cmd_reset, 2);
		vTaskDelay( 1 );
	}

	if (err == ESP_OK) {
		err = i2c_desc->write(SHTC3::m_cmd_detect, 2);
	}

	uint8_t recv_buffer[3] = { 0 };
	if (err == ESP_OK) {
		err = i2c_desc->read(recv_buffer, 3);
	}
	if (err == ESP_OK) {
		err = (SHTC3::computeCRC8(recv_buffer, 2) == recv_buffer[2]) ? ESP_OK : ESP_ERR_INVALID_CRC;
	}

	if (err == ESP_OK) {
		err = i2c_desc->write(SHTC3::m_cmd_sleep, 2);
	}

	i2c_desc->deinit();
	HardwareDescriptor::unlockI2C(i2c_desc->getPort());

	return err;
}
