/*
 * Actuator.hpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#ifndef COMPONENTS_PERIPHERAL_ACTUATOR_HPP_
#define COMPONENTS_PERIPHERAL_ACTUATOR_HPP_

#include "Peripheral.hpp"
#include "esp_err.h"

class Actuator: virtual public Peripheral {
public:
    Actuator(std::string t_name, gpio_num_t t_enable_pin = GPIO_NUM_NC, uint16_t t_turn_on_time_ms = 0,
            uint8_t t_enable_level = 1) :
            Peripheral(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level) {
    }
    virtual ~Actuator() {
    }

    virtual esp_err_t init() = 0;

    /* Set the actuator to a specified state: i.e. open, close, 5814 */
    virtual esp_err_t set(int value) = 0;

    /* Enable the actuator, when applicable, i.e. High-Z */
    virtual esp_err_t enable() = 0;

    /* Disable the actuator, when applicable, i.e. High-Z */
    virtual esp_err_t disable() = 0;

};

#endif /* COMPONENTS_PERIPHERAL_ACTUATOR_HPP_ */
