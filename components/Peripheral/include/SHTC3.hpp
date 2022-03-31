/*
 * SHTC3.hpp
 *
 *  Created on: Mar. 30, 2022
 *      Author: malalonde
 */

#ifndef SHTC3_HPP_
#define SHTC3_HPP_

#include "I2CThreadSafeDriver.hpp"

#include "esp_log.h"

#define LOG_TAG "SHTC3"

class SHTC3 {
public:
	SHTC3(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t bus_speed = 1000000,
			i2c_port_t i2c_num = I2C_NUM_MAX - 1);
	virtual ~SHTC3();

	esp_err_t update();
	float getTemperature_celsius();
	float getHumidity_pct();

private:
	I2CThreadSafeDriver *m_i2c_driver;
	float m_temperature_celsius;
	float m_humidity_pct;

	static uint8_t computeCRC8(uint8_t data[], uint16_t len);

};

#endif /* SHTC3_HPP_ */
