/*
 * I2CThreadSafeDriver.hpp
 *
 *  Created on: Mar. 22, 2022
 *      Author: ma-lalonde
 */

#ifndef I2CTHREADSAFEDRIVER_HPP_
#define I2CTHREADSAFEDRIVER_HPP_

#include <stdint.h>

#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

class I2CThreadSafeDriver {
public:
	I2CThreadSafeDriver(i2c_config_t i2c_config, i2c_port_t i2c_num, uint8_t timeout_ms = 5);
	~I2CThreadSafeDriver();

	esp_err_t masterWrite(const uint8_t address, const uint8_t data[], size_t length);
	esp_err_t masterRead(const uint8_t address, uint8_t buffer[], size_t length);

	i2c_port_t getPort();

	uint8_t getTimeout_ms();
	void setTimeout_ms(uint8_t timeout_ms);

private:
	i2c_config_t m_conf;
	i2c_port_t m_i2c_num;
	uint8_t m_timeout_ticks;

	static SemaphoreHandle_t m_counting_semaphore;
	static SemaphoreHandle_t m_i2c_mutexes[I2C_NUM_MAX];

	static void lock(i2c_port_t i2c_num);
	static void unlock(i2c_port_t i2c_num);

};

#endif /* I2CTHREADSAFEDRIVER_HPP_ */
