/*
 * ThreadSafeI2CDriver.cpp
 *
 *  Created on: Mar. 22, 2022
 *      Author: ma-lalonde
 */

#include "include/I2CThreadSafeDriver.hpp"

#include "esp_log.h"

#define LOG_TAG "I2CThreadSafeDriver"

SemaphoreHandle_t I2CThreadSafeDriver::m_counting_semaphore = xSemaphoreCreateCounting(0xFFFFFFFF, 0);
SemaphoreHandle_t I2CThreadSafeDriver::m_i2c_mutexes[I2C_NUM_MAX];

I2CThreadSafeDriver::I2CThreadSafeDriver(i2c_config_t i2c_config, uint8_t i2c_num, uint8_t timeout_ms) :
		m_conf(i2c_config), m_i2c_num(i2c_num) {

	setTimeout_ms(timeout_ms);

	I2CThreadSafeDriver::lock(0);

	if (uxSemaphoreGetCount(I2CThreadSafeDriver::m_counting_semaphore) == 0) {
		for (uint8_t i = 0; i < I2C_NUM_MAX; ++i) {
			I2CThreadSafeDriver::m_i2c_mutexes[i] = xSemaphoreCreateMutex();
		}
	}
	xSemaphoreGive(I2CThreadSafeDriver::m_counting_semaphore);

	I2CThreadSafeDriver::unlock(0);
}

I2CThreadSafeDriver::~I2CThreadSafeDriver() {
	I2CThreadSafeDriver::lock(0);

	xSemaphoreTake(I2CThreadSafeDriver::m_counting_semaphore, 0);
	if (uxSemaphoreGetCount(I2CThreadSafeDriver::m_counting_semaphore) == 0) {
		for (uint8_t i = 0; i < I2C_NUM_MAX; ++i) {
			vSemaphoreDelete(I2CThreadSafeDriver::m_i2c_mutexes[i]);
		}
	}

	I2CThreadSafeDriver::unlock(0);
}

esp_err_t I2CThreadSafeDriver::masterWrite(const uint8_t address, const uint8_t data[], size_t length) {
	I2CThreadSafeDriver::lock(m_i2c_num);
	esp_err_t err = ESP_OK, err_inst = ESP_OK;

	err = i2c_param_config(m_i2c_num, &m_conf);
	if (err == ESP_OK) {
		err_inst = i2c_driver_install(m_i2c_num, m_conf.mode, 0, 0, 0);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		err = i2c_master_write_to_device(m_i2c_num, address, data, length, m_timeout_ticks);
	}
	if (err_inst == ESP_OK) {
		i2c_driver_delete(m_i2c_num);
	}

	I2CThreadSafeDriver::unlock(m_i2c_num);

	return err & err_inst;
}

esp_err_t I2CThreadSafeDriver::masterRead(const uint8_t address, uint8_t buffer[], size_t length) {
	I2CThreadSafeDriver::lock(m_i2c_num);
	esp_err_t err = ESP_OK, err_inst = ESP_OK;

	err = i2c_param_config(m_i2c_num, &m_conf);
	if (err == ESP_OK) {
		err_inst = i2c_driver_install(m_i2c_num, m_conf.mode, 0, 0, 0);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		err = i2c_master_read_from_device(m_i2c_num, address, buffer, length, m_timeout_ticks);
	}
	if (err_inst == ESP_OK) {
		i2c_driver_delete(m_i2c_num);
	}

	I2CThreadSafeDriver::unlock(m_i2c_num);

	return err & err_inst;
}

i2c_port_t I2CThreadSafeDriver::getPort() {
	return m_i2c_num;
}

uint8_t I2CThreadSafeDriver::getTimeout_ms() {
	return ((m_timeout_ticks * 1000) / configTICK_RATE_HZ);
}
void I2CThreadSafeDriver::setTimeout_ms(uint8_t timeout_ms) {
	m_timeout_ticks = pdMS_TO_TICKS(timeout_ms);
	if (m_timeout_ticks == 0 && timeout_ms != 0) {
		m_timeout_ticks = 1;
	}
}

void I2CThreadSafeDriver::lock(uart_port_t i2cNum) {
	xSemaphoreTake(I2CThreadSafeDriver::m_i2c_mutexes[i2cNum], portMAX_DELAY);
}

void I2CThreadSafeDriver::unlock(uart_port_t i2cNum) {
	xSemaphoreGive(I2CThreadSafeDriver::m_i2c_mutexes[i2cNum]);
}

