/*
 * CSMS12.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_PERIPHERAL_CSMS12_HPP_
#define COMPONENTS_PERIPHERAL_CSMS12_HPP_

#include "Sensor.hpp"

class CSMS12: public Sensor {
public:
	CSMS12(std::string t_name, gpio_num_t t_signal_pin, gpio_num_t t_enable_pin = GPIO_NUM_NC,
			uint16_t t_turn_on_time_ms = 0, uint8_t t_enable_level = 0);
	~CSMS12();

	esp_err_t update();
};

#endif /* COMPONENTS_PERIPHERAL_CSMS12_HPP_ */
