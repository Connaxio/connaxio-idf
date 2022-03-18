/*
 * Sensor.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_PERIPHERAL_SENSOR_HPP_
#define COMPONENTS_PERIPHERAL_SENSOR_HPP_

#include <map>

#include "Peripheral.hpp"
#include "esp_err.h"

class Sensor: virtual public Peripheral {
public:
	Sensor(std::string t_name, gpio_num_t t_enable_pin = GPIO_NUM_NC, uint16_t t_turn_on_time_ms = 0,
			uint8_t t_enable_level = 1) :
			Peripheral(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level) {
	}
	virtual ~Sensor() {
	}

	virtual esp_err_t update() = 0;

	float getValue(std::string value_name);

protected:
	std::map<std::string, float> m_values;

};

#endif /* COMPONENTS_PERIPHERAL_SENSOR_HPP_ */
