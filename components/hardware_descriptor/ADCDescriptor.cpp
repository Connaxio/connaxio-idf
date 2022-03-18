/*
 * ADCDescriptor.cpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#include "ADCDescriptor.hpp"

#include <string>

#include "esp_log.h"

#define DEFAULT_VREF 1100
#define LOG_TAG "ADCDescriptor"

ADCDescriptor::ADCDescriptor(gpio_num_t t_gpio_pin, adc_bits_width_t t_resolution,
		adc_atten_t t_attenuation) {
	adc_unit_t unit = ADCDescriptor::getADCUnit(t_gpio_pin);
	m_channel = ADCDescriptor::getADCChannel(t_gpio_pin);

	if (unit == ADC_UNIT_MAX || m_channel == ADC_CHANNEL_MAX) {
		assert(0);
	}
	adc_set_data_width(unit, t_resolution);
	if (unit == ADC_UNIT_1) {
		adc1_config_channel_atten((adc1_channel_t) m_channel, t_attenuation);
	} else {
		adc2_config_channel_atten((adc2_channel_t) m_channel, t_attenuation);
	}

	esp_adc_cal_characterize(unit, t_attenuation, t_resolution, DEFAULT_VREF, &m_adc_chars);
	adc_set_clk_div(240); // 165 Ohms * 2 pF * 9 * APB_CLK (80 MHz)

}

ADCDescriptor::~ADCDescriptor() {
// TODO Auto-generated destructor stub
}

void ADCDescriptor::update() {
    m_rawValue = -1;
	ESP_LOGV(LOG_TAG, "Reading ADC channel: %i", m_channel);

	adc_power_acquire();
	if (m_adc_chars.adc_num == ADC_UNIT_1) {
		m_rawValue = adc1_get_raw((adc1_channel_t) m_channel);
	} else {
		int raw;
		if (adc2_get_raw((adc2_channel_t) m_channel, m_adc_chars.bit_width, &raw) == ESP_OK) {
			m_rawValue = raw;
		}
	}
	adc_power_release();

}

int ADCDescriptor::getRawValue(){
    return m_rawValue;
}

uint32_t ADCDescriptor::getVoltage()
{
    return esp_adc_cal_raw_to_voltage(m_rawValue, &m_adc_chars);
}

adc_channel_t ADCDescriptor::getADCChannel(gpio_num_t t_gpio_pin) {
	adc_channel_t channel = ADC_CHANNEL_MAX;
	gpio_num_t gpio_pin_tmp;

	if (t_gpio_pin >= GPIO_NUM_32 && t_gpio_pin <= GPIO_NUM_39) {
		for (uint8_t ch = ADC1_CHANNEL_0; ch < ADC1_CHANNEL_MAX; ++ch) {
			if ((adc1_pad_get_io_num((adc1_channel_t) ch, &gpio_pin_tmp) == ESP_OK)
					&& (gpio_pin_tmp == t_gpio_pin)) {
				channel = (adc_channel_t) ch;
				break;
			}
		}

	} else if (t_gpio_pin == GPIO_NUM_4 || t_gpio_pin == GPIO_NUM_2
			|| (t_gpio_pin >= GPIO_NUM_12 && t_gpio_pin <= GPIO_NUM_15)) {
		for (uint8_t ch = ADC2_CHANNEL_0; ch < ADC2_CHANNEL_MAX; ++ch) {
			if ((adc2_pad_get_io_num((adc2_channel_t) ch, &gpio_pin_tmp) == ESP_OK)
					&& (gpio_pin_tmp == t_gpio_pin)) {
				channel = (adc_channel_t) ch;
				break;
			}
		}
	}
	ESP_LOGD(LOG_TAG, "ADC channel selected: %i.", channel);
	return channel;
}

adc_unit_t ADCDescriptor::getADCUnit(gpio_num_t t_gpio_pin) {
	adc_unit_t unit = ADC_UNIT_MAX;

	if (t_gpio_pin >= GPIO_NUM_32 && t_gpio_pin <= GPIO_NUM_39) {
		unit = ADC_UNIT_1;
	} else if (t_gpio_pin == GPIO_NUM_4 || t_gpio_pin == GPIO_NUM_2
			|| (t_gpio_pin >= GPIO_NUM_12 && t_gpio_pin <= GPIO_NUM_15)) {
		unit = ADC_UNIT_2;
	}
	return unit;
}
