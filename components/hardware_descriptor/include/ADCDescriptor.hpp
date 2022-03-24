/*
 * ADCDescriptor.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_HARDWAREDESCRIPTOR_ADCDESCRIPTOR_HPP_
#define COMPONENTS_HARDWAREDESCRIPTOR_ADCDESCRIPTOR_HPP_

#include "HardwareDescriptor.hpp"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

class ADCDescriptor: public HardwareDescriptor {
public:
	ADCDescriptor(gpio_num_t gpio_pin, adc_bits_width_t t_resolution, adc_atten_t t_attenuation);
	~ADCDescriptor();

	void update();
	int getRawValue();
	uint32_t getVoltage();

	static esp_err_t getID(gpio_num_t io_num, adc_unit_t &unit_out, adc_channel_t &channel_out);

private:
	adc_channel_t m_channel;
	esp_adc_cal_characteristics_t m_adc_chars;
	int m_rawValue;

};

#endif /* COMPONENTS_HARDWAREDESCRIPTOR_ADCDESCRIPTOR_HPP_ */
