/*
 * ActuatorSwitch.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_PERIPHERAL_ACTUATORSWITCH_HPP_
#define COMPONENTS_PERIPHERAL_ACTUATORSWITCH_HPP_

#include "Actuator.hpp"

class ActuatorSwitch: public Actuator {
public:
	ActuatorSwitch(std::string t_name);
	virtual ~ActuatorSwitch();
};

#endif /* COMPONENTS_PERIPHERAL_ACTUATORSWITCH_HPP_ */
