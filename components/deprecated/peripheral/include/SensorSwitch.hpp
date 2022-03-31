/*
 * SensorSwitch.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_PERIPHERAL_SENSORSWITCH_HPP_
#define COMPONENTS_PERIPHERAL_SENSORSWITCH_HPP_

#include "Sensor.hpp"

class SensorSwitch: public Sensor {
public:
	SensorSwitch(std::string t_name);
	virtual ~SensorSwitch();
};

#endif /* COMPONENTS_PERIPHERAL_SENSORSWITCH_HPP_ */
