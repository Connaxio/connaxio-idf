/*
 * ADS1119IPWR.h
 *
 *  Created on: Jan. 2, 2022
 *      Author: malalonde
 */

#ifndef COMPONENTS_PERIPHERAL_ADS1119IPWR_H_
#define COMPONENTS_PERIPHERAL_ADS1119IPWR_H_

#include "Sensor.hpp"

class ADS1119: public Sensor {
public:
    ADS1119(std::string t_name, gpio_num_t t_sda_gpio_num, gpio_num_t t_scl_gpio_num, gpio_num_t t_enable_pin =
            GPIO_NUM_NC, uint16_t t_turn_on_time_ms = 0, uint8_t t_enable_level = 0);
    virtual ~ADS1119();

    esp_err_t init();

    esp_err_t update();

    esp_err_t updateChannel(uint8_t channel);

private:
    uint8_t m_address = 0x40;
    static const uint8_t m_cmd_reset;
    static const uint8_t m_cmd_start;
    static const uint8_t m_cmd_pwrdn;
    static const uint8_t m_cmd_rdata;
    static const uint8_t m_cmd_rreg_conf;
    static const uint8_t m_cmd_rreg_status;
    static const uint8_t m_cmd_wreg;
    uint16_t m_offset;

};

#endif /* COMPONENTS_PERIPHERAL_ADS1119IPWR_H_ */
