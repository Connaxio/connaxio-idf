/*
 * SHTC3.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_PERIPHERAL_SHTC3_HPP_
#define COMPONENTS_PERIPHERAL_SHTC3_HPP_

#include "Sensor.hpp"


class SHTC3: public Sensor {
public:
	SHTC3(std::string t_name, gpio_num_t t_sda_gpio_num, gpio_num_t t_scl_gpio_num, gpio_num_t t_enable_pin =
			GPIO_NUM_NC, uint16_t t_turn_on_time_ms = 0, uint8_t t_enable_level = 0);
	~SHTC3();

	esp_err_t init();

	esp_err_t update();




private:
	uint8_t m_address = 0x70;
	static const uint8_t m_cmd_wakeup[2];
	static const uint8_t m_cmd_sleep[2];
	static const uint8_t m_cmd_meas[2];
	static const uint8_t m_cmd_detect[2];
	static const uint8_t m_cmd_reset[2];


	static uint8_t computeCRC8(uint8_t data[], uint16_t len);
};

#endif /* COMPONENTS_PERIPHERAL_SHTC3_HPP_ */
