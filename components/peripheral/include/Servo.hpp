/*
 * Servo.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_PERIPHERAL_SERVO_HPP_
#define COMPONENTS_PERIPHERAL_SERVO_HPP_

#include "Actuator.hpp"

class Servo: public Actuator {
public:
	Servo(std::string t_name);
	~Servo();
};

#endif /* COMPONENTS_PERIPHERAL_SERVO_HPP_ */
