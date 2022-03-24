/*
 * UARTThreadSafeDriver.hpp
 *
 *  Created on: Mar. 22, 2022
 *      Author: ma-lalonde
 */

#ifndef UARTTHREADSAFEDRIVER_HPP_
#define UARTTHREADSAFEDRIVER_HPP_

#include <stdint.h>

#include "driver/uart.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

class UARTThreadSafeDriver {
public:
	UARTThreadSafeDriver(uart_config_t config, uart_port_t uart_num, gpio_num_t tx_io_num,
			gpio_num_t rx_io_num, gpio_num_t cts_io_num = (gpio_num_t) UART_PIN_NO_CHANGE, gpio_num_t rts_io_num =
					(gpio_num_t) UART_PIN_NO_CHANGE, uint8_t timeout_ms = 10);
	~UARTThreadSafeDriver();

	esp_err_t write(const uint8_t data[], size_t length);
	esp_err_t read(uint8_t buffer[], size_t length);
	esp_err_t poll(const uint8_t write_data[], size_t write_length, uint8_t read_buffer[], size_t read_length);

	uart_port_t getPort();
	esp_err_t setBaudrate(uint32_t baudrate);

	uint8_t getTimeout_ms();
	void setTimeout_ms(uint8_t timeout_ms);

private:
	uart_config_t m_conf;
	uart_port_t m_uart_num;
	gpio_num_t m_tx_io_num;
	gpio_num_t m_rx_io_num;
	gpio_num_t m_cts_io_num;
	gpio_num_t m_rts_io_num;
	uint8_t m_timeout_ticks;

	static QueueHandle_t m_uart_queue_handle;

	static SemaphoreHandle_t m_countingSemaphore;
	static SemaphoreHandle_t m_uart_mutexes[UART_NUM_MAX];

	static void lock(uart_port_t uart_num);
	static void unlock(uart_port_t uart_num);

};

#endif /* UARTTHREADSAFEDRIVER_HPP_ */
