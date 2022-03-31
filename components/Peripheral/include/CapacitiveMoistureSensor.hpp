/*
 * CapacitiveMoistureSensor.hpp
 *
 *  Created on: Mar. 28, 2022
 *      Author: malalonde
 */

#ifndef CAPACITIVEMOISTURESENSOR_HPP_
#define CAPACITIVEMOISTURESENSOR_HPP_

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

class CapacitiveMoistureSensor {
public:
	CapacitiveMoistureSensor(gpio_num_t io_num, float voltage_0 = 2.2f, float voltage_100 = 1.0f,
			float moisture_calibration_factor = 3.0f);
	virtual ~CapacitiveMoistureSensor();

	esp_err_t update();
	float getMoisture_pct();

private:
	float m_moisture_pct;
	adc_unit_t m_adc_unit;
	adc_channel_t m_adc_channel;
	esp_adc_cal_characteristics_t m_adc_chars;
	float m_moisture_calibration_factor;
	float m_moisture_coeff_0;
	float m_moisture_coeff_1;

	static esp_err_t getADCUnitandChannel(gpio_num_t io_num, adc_unit_t &unit_out,
			adc_channel_t &channel_out);

	/*	\brief get Order 0 and Order 1 coefficients for a linear line between 2 points
	 *
	 */
	static esp_err_t getLinearCoefficientsFromPoints(float x1, float y1, float x2, float y2, float &coeff_0,
			float &coeff_1);

};

#endif /* CAPACITIVEMOISTURESENSOR_HPP_ */
