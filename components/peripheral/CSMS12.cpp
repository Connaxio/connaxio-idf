/*
 * CSMS12.cpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#include "CSMS12.hpp"

#include <math.h>
#include <string>

#include "ADCDescriptor.hpp"
#include "esp_log.h"

#define LOG_TAG "CSMS12"

#define TOPIC_MOISTURE  "moisture"


/* Calibration  from test. Linear from 10% to 100%. 0% misleading. */
#define P1_READING		1840
#define P1_PERCENTAGE	10.0f
#define P2_READING		1050
#define P2_PERCENTAGE	100.0f

#define A ( (P2_PERCENTAGE - P1_PERCENTAGE) / (P2_READING - P1_READING) )
#define B ( (P1_PERCENTAGE) - (A * P1_READING) )

#define MIN_VALUE 950
#define MAX_VALUE 2620

CSMS12::CSMS12(std::string t_name, gpio_num_t t_signal_pin, gpio_num_t t_enable_pin,
		uint16_t t_turn_on_time_ms, uint8_t t_enable_level) :
		Peripheral(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level), Sensor(t_name, t_enable_pin,
				t_turn_on_time_ms, t_enable_level) {

	m_hardware_descriptor = new ADCDescriptor(t_signal_pin, ADC_WIDTH_BIT_12, ADC_ATTEN_DB_11);
	m_values[TOPIC_MOISTURE] = NAN;

}

CSMS12::~CSMS12() {
	delete (m_hardware_descriptor);
}

esp_err_t CSMS12::update() {
	esp_err_t err = ESP_OK;
	uint16_t v_moist = 0;
	uint32_t v_tmp;

	this->turnONAndWait();

	//Read ADC
	for (uint8_t i = 0; i < 16; ++i) {
	    static_cast<ADCDescriptor*>(m_hardware_descriptor)->update();
		v_tmp = static_cast<ADCDescriptor*>(m_hardware_descriptor)->getVoltage();
		if (v_tmp != -1) {
			v_moist += v_tmp;
		} else {
			--i;
		}
	}

	m_values["moisture"] = 1.0f * A * v_moist + B;
	ESP_LOGD(LOG_TAG, "%s Updated value: %f", this->getName().c_str(), m_values["moisture"]);

	if (m_values[TOPIC_MOISTURE] < 0.0f) {
		m_values[TOPIC_MOISTURE] = 0.0f;
	} else if (m_values[TOPIC_MOISTURE] > 100.0f) {
		m_values[TOPIC_MOISTURE] = 100.0f;
	}

	if ((v_moist < (1.1f * MIN_VALUE)) && (v_moist > (0.9f * MAX_VALUE))) {
		err = ESP_OK;
	} else {
		err = ESP_FAIL;
	}

	this->turnOFF();

	return err;

}
