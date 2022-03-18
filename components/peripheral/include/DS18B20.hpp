/*
 * DS18B20.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_PERIPHERAL_DS18B20_HPP_
#define COMPONENTS_PERIPHERAL_DS18B20_HPP_

#include "Sensor.hpp"

class DS18B20: public Sensor {
public:
	DS18B20(std::string t_name, gpio_num_t t_tx_gpio_num, gpio_num_t t_rx_gpio_num, gpio_num_t t_enable_pin =
			GPIO_NUM_NC, uint16_t t_turn_on_time_ms = 0, uint8_t t_enable_level = 0);
	~DS18B20();

	esp_err_t init();

	esp_err_t update();

private:

};

#endif /* COMPONENTS_PERIPHERAL_DS18B20_HPP_ */
