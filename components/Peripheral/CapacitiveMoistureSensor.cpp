/*
 * CapacitiveMoistureSensor.cpp
 *
 *  Created on: Mar. 28, 2022
 *      Author: malalonde
 */

#include "CapacitiveMoistureSensor.hpp"

#include <cmath>

#include "esp_adc_cal.h"
#include "esp_log.h"

#define LOG_TAG "CapacitiveMoistureSensor"

CapacitiveMoistureSensor::CapacitiveMoistureSensor(gpio_num_t io_num, float voltage_0, float voltage_100,
		float moisture_calibration_factor) :
		m_moisture_calibration_factor(moisture_calibration_factor) {
	assert(voltage_0 >= 0.0f && voltage_0 < 3.3f);
	assert(voltage_100 >= 0.0f && voltage_100 < 3.3f && voltage_100 < voltage_0);
	assert(moisture_calibration_factor >= 1 && moisture_calibration_factor <= 5);

	// Calibrate moisture sensor curve
	assert(
			CapacitiveMoistureSensor::getLinearCoefficientsFromPoints(
					(float) pow(voltage_0, -m_moisture_calibration_factor), 0,
					(float) pow(voltage_100, -m_moisture_calibration_factor), 100, m_moisture_coeff_0,
					m_moisture_coeff_1) == ESP_OK);

	// Find ADC unit and channel for the IO pin
	assert(CapacitiveMoistureSensor::getADCUnitandChannel(io_num, m_adc_unit, m_adc_channel) == ESP_OK);
	if (m_adc_unit == ADC_UNIT_1) {
		adc1_config_channel_atten((adc1_channel_t) m_adc_channel, ADC_ATTEN_DB_11);
	} else if (m_adc_unit == ADC_UNIT_2) {
		adc2_config_channel_atten((adc2_channel_t) m_adc_channel, ADC_ATTEN_DB_11);
	}

	esp_adc_cal_characterize(m_adc_unit, ADC_ATTEN_DB_11, (adc_bits_width_t) ADC_WIDTH_BIT_DEFAULT, 1100,
			&m_adc_chars);

}

CapacitiveMoistureSensor::~CapacitiveMoistureSensor() {
	// TODO Auto-generated destructor stub
}

esp_err_t CapacitiveMoistureSensor::update() {
	esp_err_t err = ESP_OK;
	int adc_raw;
	adc_power_acquire();
	if (m_adc_unit == ADC_UNIT_1) {
		adc_raw = adc1_get_raw((adc1_channel_t) m_adc_channel);
	} else if (m_adc_unit == ADC_UNIT_2) {
		err = adc2_get_raw((adc2_channel_t) m_adc_channel, (adc_bits_width_t) ADC_WIDTH_BIT_DEFAULT,
				&adc_raw);
	}
	adc_power_release();

	uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_raw, &m_adc_chars);
	ESP_LOGD(LOG_TAG, "ADC voltage (mv):\t%i", voltage_mv);

	m_moisture_pct = m_moisture_coeff_1 * pow(1.0f * voltage_mv / 1000.0f, -m_moisture_calibration_factor)
			+ m_moisture_coeff_0;

	return err;
}
float CapacitiveMoistureSensor::getMoisture_pct() {
	return m_moisture_pct;
}

esp_err_t CapacitiveMoistureSensor::getADCUnitandChannel(gpio_num_t io_num, adc_unit_t &unit_out,
		adc_channel_t &channel_out) {
	gpio_num_t gpio_pin_tmp = GPIO_NUM_NC;

	for (uint8_t ch = ADC1_CHANNEL_0; ch < ADC1_CHANNEL_MAX && gpio_pin_tmp != io_num; ++ch) {
		adc1_pad_get_io_num((adc1_channel_t) ch, &gpio_pin_tmp);
		if (gpio_pin_tmp == io_num) {
			channel_out = (adc_channel_t) ch;
			unit_out = ADC_UNIT_1;
		}
	}

	for (uint8_t ch = ADC2_CHANNEL_0; ch < ADC2_CHANNEL_MAX && gpio_pin_tmp != io_num; ++ch) {
		adc2_pad_get_io_num((adc2_channel_t) ch, &gpio_pin_tmp);
		if (gpio_pin_tmp == io_num) {
			channel_out = (adc_channel_t) ch;
			unit_out = ADC_UNIT_2;
		}
	}

	if (gpio_pin_tmp == io_num) {
		return ESP_OK;
	} else {
		return ESP_FAIL;
	}
}

esp_err_t CapacitiveMoistureSensor::getLinearCoefficientsFromPoints(float x1, float y1, float x2, float y2,
		float &coeff_0, float &coeff_1) {
	if (x1 == x2) {
		return ESP_FAIL;
	} else {
		coeff_1 = (y2 - y1) / (x2 - x1);
		coeff_0 = (x2 * y1 - x1 * y2) / (x2 - x1);

		return ESP_OK;
	}
}

