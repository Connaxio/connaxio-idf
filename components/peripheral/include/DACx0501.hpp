/*
 * DAC7050IZDGSR.hpp
 *
 *  Created on: Jan. 3, 2022
 *      Author: malalonde
 */

#ifndef COMPONENTS_PERIPHERAL_DAC7050IZDGSR_HPP_
#define COMPONENTS_PERIPHERAL_DAC7050IZDGSR_HPP_

#include "Actuator.hpp"

class DACx0501: public Actuator {
public:
    DACx0501(std::string t_name, gpio_num_t t_sda_gpio_num, gpio_num_t t_scl_gpio_num, gpio_num_t t_enable_pin =
            GPIO_NUM_NC, uint16_t t_turn_on_time_ms = 0, uint8_t t_enable_level = 0);
    virtual ~DACx0501();

    esp_err_t init();

    /*Sets the voltage output.
     *
     * value: output voltage (mV), from 0 to VDD
     */
    esp_err_t set(int value);
    esp_err_t enable();
    esp_err_t disable();

    uint8_t getBits();

private:
    uint8_t m_address = 0x49;
    static const uint8_t m_cmd_noop;
    static const uint8_t m_cmd_devid;
    static const uint8_t m_cmd_sync;
    static const uint8_t m_cmd_config;
    static const uint8_t m_cmd_gain;
    static const uint8_t m_cmd_trigger;
    static const uint8_t m_cmd_status;
    static const uint8_t m_cmd_dacdata;

    uint8_t m_bits = 0;
};

#endif /* COMPONENTS_PERIPHERAL_DAC7050IZDGSR_HPP_ */
