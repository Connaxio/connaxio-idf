/*
 * Peripheral.cpp
 *
 *  Created on: Sep 10, 2021
 *      Author: Marc-Antoine
 */
#include "Peripheral.hpp"
#include "driver/gpio.h"

Peripheral::Peripheral(std::string t_name, gpio_num_t t_enable_pin, uint16_t t_turn_on_time_ms,
		uint8_t t_enable_level) :
		m_name(t_name), m_enable_pin(t_enable_pin), m_turn_on_time_ms(t_turn_on_time_ms), m_enable_level(
				t_enable_level) {
	m_hardware_descriptor = nullptr;

	if (m_enable_pin >= 0) {
		gpio_config_t gpio_conf = { .pin_bit_mask = (1ULL << m_enable_pin), .mode = GPIO_MODE_OUTPUT_OD,
				.pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type =
						GPIO_INTR_DISABLE };
		gpio_config(&gpio_conf);
		this->turnOFF();
	}

}

void Peripheral::turnON() {
	if (m_enable_pin >= 0) {
		gpio_set_level(m_enable_pin, (m_enable_level > 0));
	}
}

void Peripheral::turnONAndWait() {
	this->turnON();
	if (m_turn_on_time_ms > 0) {
		vTaskDelay(pdMS_TO_TICKS(m_turn_on_time_ms));
	}
}

void Peripheral::turnOFF() {
	if (m_enable_pin >= 0) {
		gpio_set_level(m_enable_pin, (m_enable_level == 0));
	}
}
