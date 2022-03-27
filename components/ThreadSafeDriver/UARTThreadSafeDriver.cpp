/*
 * UARTThreadSafeDriver.cpp
 *
 *  Created on: Mar. 22, 2022
 *      Author: ma-lalonde
 */

#include "UARTThreadSafeDriver.hpp"

#include <math.h>

#include "esp_log.h"

#define LOG_TAG "UARTThreadSafeDriver"

SemaphoreHandle_t UARTThreadSafeDriver::m_counting_semaphore = xSemaphoreCreateCounting(0xFFFFFFFF, 0);
SemaphoreHandle_t UARTThreadSafeDriver::m_uart_mutexes[UART_NUM_MAX];

UARTThreadSafeDriver::UARTThreadSafeDriver(uart_config_t config, uart_port_t uart_num, gpio_num_t tx_io_num,
		gpio_num_t rx_io_num, gpio_num_t cts_io_num, gpio_num_t rts_io_num, uint8_t timeout_ms) :
		m_conf(config), m_uart_num(uart_num), m_tx_io_num(tx_io_num), m_rx_io_num(rx_io_num), m_cts_io_num(
				cts_io_num), m_rts_io_num(rts_io_num) {
	setTimeout_ms(timeout_ms);

	if (uxSemaphoreGetCount(UARTThreadSafeDriver::m_counting_semaphore) == 0) {
		for (uint8_t i = 0; i < UART_NUM_MAX; ++i) {
			UARTThreadSafeDriver::m_uart_mutexes[i] = xSemaphoreCreateMutex();
		}
	}
	xSemaphoreGive(UARTThreadSafeDriver::m_counting_semaphore);
}

UARTThreadSafeDriver::~UARTThreadSafeDriver() {
	xSemaphoreTake(UARTThreadSafeDriver::m_counting_semaphore, 0);
	if (uxSemaphoreGetCount(UARTThreadSafeDriver::m_counting_semaphore) == 0) {
		for (uint8_t i = 0; i < UART_NUM_MAX; ++i) {
			vSemaphoreDelete(UARTThreadSafeDriver::m_uart_mutexes[i]);
		}
	}
}

esp_err_t UARTThreadSafeDriver::write(const uint8_t data[], size_t length) {
	UARTThreadSafeDriver::lock(m_uart_num);
	esp_err_t err = ESP_OK, err_inst = ESP_OK;
	int bytes_written = 0;
	size_t max_size = 2 * length > SOC_UART_FIFO_LEN + 1 ? 2 * length : SOC_UART_FIFO_LEN + 1;

	err_inst = uart_driver_install(m_uart_num, SOC_UART_FIFO_LEN + 1, max_size, 0, NULL, 0);
	if (err == ESP_OK && err_inst == ESP_OK) {
		err = uart_param_config(m_uart_num, &m_conf);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		err = uart_set_pin(m_uart_num, m_tx_io_num, m_rx_io_num, m_rts_io_num, m_cts_io_num);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		bytes_written = uart_write_bytes(m_uart_num, data, length);
		err = uart_wait_tx_done(m_uart_num, m_timeout_ticks);
	}
	if (err == ESP_OK && err_inst == ESP_OK && bytes_written != length) {
		err = ESP_FAIL;
	}
	if (err_inst == ESP_OK) {
		err_inst = uart_driver_delete(m_uart_num);
	}

	UARTThreadSafeDriver::unlock(m_uart_num);

	return err & err_inst;
}

esp_err_t UARTThreadSafeDriver::poll(const uint8_t write_data[], size_t write_length, uint8_t read_buffer[],
		size_t read_length) {
	UARTThreadSafeDriver::lock(m_uart_num);
	esp_err_t err = ESP_OK, err_inst = ESP_OK;
	int bytes_written = 0, bytes_read = 0;
	size_t max_size_read = 2 * read_length > SOC_UART_FIFO_LEN + 1 ? 2 * read_length : SOC_UART_FIFO_LEN + 1;
	size_t max_size_write =
			2 * write_length > SOC_UART_FIFO_LEN + 1 ? 2 * write_length : SOC_UART_FIFO_LEN + 1;

	err_inst = uart_driver_install(m_uart_num, max_size_read, max_size_write, 0, NULL, 0);
	if (err == ESP_OK && err_inst == ESP_OK) {
		err = uart_param_config(m_uart_num, &m_conf);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		err = uart_set_pin(m_uart_num, m_tx_io_num, m_rx_io_num, m_rts_io_num, m_cts_io_num);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		err = uart_flush(m_uart_num);	// Prepare receive buffer for polling response
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		bytes_written = uart_write_bytes(m_uart_num, write_data, write_length);
		err = uart_wait_tx_done(m_uart_num, m_timeout_ticks);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		bytes_read = uart_read_bytes(m_uart_num, read_buffer, read_length, m_timeout_ticks);
	}
	if (err == ESP_OK && err_inst == ESP_OK && (bytes_written != write_length || bytes_read != read_length)) {
		err = ESP_FAIL;
	}
	if (err_inst == ESP_OK) {
		err_inst = uart_driver_delete(m_uart_num);
	}

	UARTThreadSafeDriver::unlock(m_uart_num);

	return err & err_inst;

}

uart_port_t UARTThreadSafeDriver::getPort() {
	return m_uart_num;
}

uint8_t UARTThreadSafeDriver::getTimeout_ms() {
	return ((m_timeout_ticks * 1000) / configTICK_RATE_HZ);
}

void UARTThreadSafeDriver::setTimeout_ms(uint8_t timeout_ms) {
	m_timeout_ticks = pdMS_TO_TICKS(timeout_ms);
	if (m_timeout_ticks == 0 && timeout_ms != 0) {
		m_timeout_ticks = 1;
	}
}

void UARTThreadSafeDriver::lock(uart_port_t uart_num) {
	xSemaphoreTake(UARTThreadSafeDriver::m_uart_mutexes[uart_num], portMAX_DELAY);
}

void UARTThreadSafeDriver::unlock(uart_port_t uart_num) {
	xSemaphoreGive(UARTThreadSafeDriver::m_uart_mutexes[uart_num]);
}

esp_err_t UARTThreadSafeDriver::setBaudrate(uint32_t baudrate) {
	m_conf.baud_rate = baudrate;
	return ESP_OK;
}
