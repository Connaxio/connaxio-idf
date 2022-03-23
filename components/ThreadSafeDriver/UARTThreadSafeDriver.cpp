/*
 * UARTThreadSafeDriver.cpp
 *
 *  Created on: Mar. 22, 2022
 *      Author: ma-lalonde
 */

#include "UARTThreadSafeDriver.hpp"

#include <math.h>

#include "driver/uart.h"
#include "esp_log.h"

#define LOG_TAG "UARTThreadSafeDriver"

QueueHandle_t UARTThreadSafeDriver::m_uart_queue_handle;

UARTThreadSafeDriver::UARTThreadSafeDriver(uart_config_t config, uart_port_t uart_num, gpio_num_t tx_io_num,
		gpio_num_t rx_io_num, gpio_num_t cts_io_num, gpio_num_t rts_io_num) :
		m_conf(config), m_uart_num(uart_num), m_tx_io_num(tx_io_num), m_rx_io_num(rx_io_num), m_cts_io_num(
				cts_io_num), m_rts_io_num(rts_io_num) {

	UARTThreadSafeDriver::lock(0);

	if (uxSemaphoreGetCount(UARTThreadSafeDriver::m_countingSemaphore) == 0) {
		for (uint8_t i = 0; i < UART_NUM_MAX; ++i) {
			UARTThreadSafeDriver::m_uartMutexes[i] = xSemaphoreCreateMutex();
		}
	}
	xSemaphoreGive(UARTThreadSafeDriver::m_countingSemaphore);

	UARTThreadSafeDriver::unlock(0);

}

UARTThreadSafeDriver::~UARTThreadSafeDriver() {
	UARTThreadSafeDriver::lock(0);

	xSemaphoreTake(UARTThreadSafeDriver::m_countingSemaphore, 0);
	if (uxSemaphoreGetCount(UARTThreadSafeDriver::m_countingSemaphore) == 0) {
		for (uint8_t i = 0; i < UART_NUM_MAX; ++i) {
			vSemaphoreDelete(UARTThreadSafeDriver::m_uartMutexes[i]);
		}
	}

	UARTThreadSafeDriver::unlock(0);
}

esp_err_t UARTThreadSafeDriver::write(const uint8_t data[], size_t length) {
	UARTThreadSafeDriver::lock(m_uart_num);
	esp_err_t err = ESP_OK, err_inst = ESP_OK;
	int bytes_written = 0;
	size_t max_size = 2 * length > SOC_UART_FIFO_LEN + 1 ? 2 * length : SOC_UART_FIFO_LEN + 1;

	err_inst = uart_driver_install(m_uart_num, SOC_UART_FIFO_LEN + 1, max_size, 10,
			&UARTThreadSafeDriver::m_uart_queue_handle, 0);
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

esp_err_t UARTThreadSafeDriver::read(uint8_t buffer[], size_t length) {
	UARTThreadSafeDriver::lock(m_uart_num);
	esp_err_t err = ESP_OK, err_inst = ESP_OK;
	int bytes_read = 0;
	size_t max_size = 2 * length > SOC_UART_FIFO_LEN + 1 ? 2 * length : SOC_UART_FIFO_LEN + 1;

	err_inst = uart_driver_install(m_uart_num, max_size, 0, 10, &UARTThreadSafeDriver::m_uart_queue_handle,
			0);
	if (err == ESP_OK && err_inst == ESP_OK) {
		err = uart_param_config(m_uart_num, &m_conf);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		err = uart_set_pin(m_uart_num, m_tx_io_num, m_rx_io_num, m_rts_io_num, m_cts_io_num);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		bytes_read = uart_read_bytes(m_uart_num, buffer, length, m_timeout_ticks);
	}
	if (err == ESP_OK && err_inst == ESP_OK && bytes_read != length) {
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

	err_inst = uart_driver_install(m_uart_num, max_size_read, max_size_write, 10,
			&UARTThreadSafeDriver::m_uart_queue_handle, 0);
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
		bytes_written = uart_write_bytes(m_uart_num, data, length);
		uart_wait_tx_done(m_uart_num, pdMS_TO_TICKS(timeout_ms));
		err = uart_wait_tx_done(m_uart_num, m_timeout_ticks);
	}
	if (err == ESP_OK && err_inst == ESP_OK) {
		bytes_read = uart_read_bytes(m_uart_num, buffer, length, m_timeout_ticks);
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

void UARTThreadSafeDriver::lock(uart_port_t uartNum) {
	xSemaphoreTake(HardwareDescriptor::m_UARTMutexes[uartNum], portMAX_DELAY);
}

void UARTThreadSafeDriver::unlock(uart_port_t uartNum) {
	xSemaphoreGive(HardwareDescriptor::m_UARTMutexes[uartNum]);
}
