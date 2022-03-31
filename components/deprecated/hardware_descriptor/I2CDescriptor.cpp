/*
 * I2CDescriptor.cpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#include "I2CDescriptor.hpp"

#include "esp_log.h"

#define LOG_TAG "I2CDescriptor"

I2CDescriptor::I2CDescriptor(gpio_num_t sda_gpio_num, gpio_num_t scl_gpio_num, uint32_t clk_speed,
        int t_i2c_num, uint8_t t_address) :
        m_port(t_i2c_num), m_address(t_address) {
    m_conf.mode = I2C_MODE_MASTER;
    m_conf.sda_io_num = sda_gpio_num;
    m_conf.scl_io_num = scl_gpio_num;
    m_conf.sda_pullup_en = true;
    m_conf.scl_pullup_en = true;
    m_conf.master.clk_speed = clk_speed;
    m_conf.clk_flags = 0x0000;

    ESP_LOGD(LOG_TAG, "Init Clock tags: 0x%04X", m_conf.clk_flags);
}

I2CDescriptor::~I2CDescriptor() {
    // TODO Auto-generated destructor stub
}

esp_err_t I2CDescriptor::write(const uint8_t data[], size_t length) {
    return i2c_master_write_to_device(m_port, m_address, data, length, pdMS_TO_TICKS(10) + 1);
}

esp_err_t I2CDescriptor::read(uint8_t buffer[], size_t length) {
    return i2c_master_read_from_device(m_port, m_address, buffer, length, pdMS_TO_TICKS(10) + 1);
}

i2c_port_t I2CDescriptor::getPort() {
    return m_port;
}

esp_err_t I2CDescriptor::init() {
    esp_err_t err = i2c_param_config(m_port, &m_conf);
    ESP_LOGD(LOG_TAG, "Reinit Clock tags: 0x%04X", m_conf.clk_flags);
    if (err == ESP_OK) {
        err = i2c_driver_install(m_port, m_conf.mode, 0, 0, 0);
    }
    return err;
}

void I2CDescriptor::deinit() {
    i2c_driver_delete(m_port);
}
