/*
 * DS18B20_UART.hpp
 *
 *  Created on: Mar. 25, 2022
 *      Author: malalonde
 */

#ifndef DS18B20_UART_HPP_
#define DS18B20_UART_HPP_

#include "OneWireUARTThreadSafeDriver.hpp"

#include <array>

#define DS18B20_FAMILY_ID 0x28

class DS18B20_UART {
public:
	DS18B20_UART(gpio_num_t tx_io_num, gpio_num_t rx_io_num, std::array<uint8_t, 8> address, uint8_t resolution_bits = 12, uart_port_t uart_num = UART_NUM_1);
	virtual ~DS18B20_UART();

	esp_err_t initialize();
	esp_err_t startConversion();
	esp_err_t startConversionAll();
	esp_err_t read();
	esp_err_t update();
	float getTemperature_celsius();
	esp_err_t setResolution(uint8_t resolution_bits);

private:
	std::array<uint8_t, 8> m_address;
	OneWireUARTThreadSafeDriver *m_ow_driver;
	float m_temperature_celsius;
	uint8_t m_resolution_bits;

	esp_err_t setResolution();
};



#endif /* DS18B20_UART_HPP_ */
