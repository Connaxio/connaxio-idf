/*
 * I2CDescriptor.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_HARDWAREDESCRIPTOR_I2CDESCRIPTOR_HPP_
#define COMPONENTS_HARDWAREDESCRIPTOR_I2CDESCRIPTOR_HPP_

#include "HardwareDescriptor.hpp"
#include "driver/i2c.h"

#define HIGH_SPEED_I2C_BUS I2C_NUM

class I2CDescriptor: public HardwareDescriptor {
public:
	I2CDescriptor(gpio_num_t sda_gpio_num, gpio_num_t scl_gpio_num, uint32_t clk_speed, int i2c_num, uint8_t address);
	~I2CDescriptor();

	esp_err_t init();
	void deinit();
	i2c_port_t getPort();

	esp_err_t write(const uint8_t data[], size_t length);
	esp_err_t read(uint8_t buffer[], size_t length);

private:
	i2c_config_t m_conf;
	i2c_port_t m_port;
	uint8_t m_address;

};

#endif /* COMPONENTS_HARDWAREDESCRIPTOR_I2CDESCRIPTOR_HPP_ */
