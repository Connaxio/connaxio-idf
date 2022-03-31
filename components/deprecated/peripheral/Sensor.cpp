/*
 * Sensor.cpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#include "Sensor.hpp"

#include <math.h>

float Sensor::getValue(std::string value_name) {
		auto item = m_values.find(value_name);
		if (item != m_values.end()) {
			return item->second;
		} else {
			return NAN;
		}
	}
