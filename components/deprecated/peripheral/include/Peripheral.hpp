#ifndef _PERIPHERAL_H_
#define _PERIPHERAL_H_

#include <string>
#include <stdint.h>
#include "driver/gpio.h"
#include "HardwareDescriptor.hpp"

class Peripheral {
public:
	Peripheral(std::string t_name, gpio_num_t t_enable_pin = GPIO_NUM_NC, uint16_t t_turn_on_time_ms = 0,
			uint8_t t_enable_level = 1);
	virtual ~Peripheral() {
	}

	std::string getName() {
		return m_name;
	}

	virtual esp_err_t init() = 0;

protected:
	HardwareDescriptor *m_hardware_descriptor;

	void turnON();
	void turnONAndWait();
	void turnOFF();

private:
	std::string m_name;
	gpio_num_t m_enable_pin;
	uint16_t m_turn_on_time_ms;
	uint8_t m_enable_level;
};

#endif /* _PERIPHERAL_H_ */

