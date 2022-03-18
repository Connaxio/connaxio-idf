/*
 * DAC70501.cpp
 *
 *  Created on: Jan. 3, 2022
 *      Author: malalonde
 */

#include "DACx0501.hpp"

#include "esp_log.h"

#include "I2CDescriptor.hpp"

#define DACx0501_I2C_NUM    I2C_NUM_0
#define LOG_TAG "DACx0501"

const uint8_t DACx0501::m_cmd_noop = 0x00;
const uint8_t DACx0501::m_cmd_devid = 0x01;
const uint8_t DACx0501::m_cmd_sync = 0x02;
const uint8_t DACx0501::m_cmd_config = 0x3;
const uint8_t DACx0501::m_cmd_gain = 0x04;
const uint8_t DACx0501::m_cmd_trigger = 0x05;
const uint8_t DACx0501::m_cmd_status = 0x07;
const uint8_t DACx0501::m_cmd_dacdata = 0x08;

DACx0501::DACx0501(std::string t_name, gpio_num_t t_sda_gpio_num, gpio_num_t t_scl_gpio_num,
        gpio_num_t t_enable_pin, uint16_t t_turn_on_time_ms, uint8_t t_enable_level) :
        Peripheral(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level), Actuator(t_name, t_enable_pin,
                t_turn_on_time_ms, t_enable_level) {
    m_hardware_descriptor = new I2CDescriptor(t_sda_gpio_num, t_scl_gpio_num, 1000000, DACx0501_I2C_NUM,
            m_address);

}

DACx0501::~DACx0501() {
    // TODO Auto-generated destructor stub
}

esp_err_t DACx0501::init() {
    esp_err_t err = ESP_OK;
    uint8_t readout[2] = { 0, 0 };
    /* As per datasheet, at 3.3V, VREFIO <= 0.5*(VDD-0.2).
     * There is also a drop somewhere inside limiting output voltage to 2.05V.
     */
    uint8_t cmd_gain[3] = { DACx0501::m_cmd_gain, 0x01, 0x01 };

    I2CDescriptor *i2c_desc = static_cast<I2CDescriptor*>(m_hardware_descriptor);
    HardwareDescriptor::lockI2C(i2c_desc->getPort());
    err = i2c_desc->init();

    if (err == ESP_OK) {
        err = i2c_desc->write(&DACx0501::m_cmd_devid, 1);
        if (err == ESP_OK) {
            err = i2c_desc->read(readout, 2);
            if (err == ESP_OK) {
                if ((readout[0] | 0x00) == 0) {
                    m_bits = 16;
                } else if ((readout[0] & 0x10) != 0) {
                    m_bits = 14;
                } else if ((readout[0] & 0x20) != 0) {
                    m_bits = 12;
                }

                ESP_LOGD(LOG_TAG, "Bits: %i", m_bits);
            }
        }
    }
    if (err == ESP_OK) {
        err = i2c_desc->write(cmd_gain, 3);
    }

    i2c_desc->deinit();
    HardwareDescriptor::unlockI2C(i2c_desc->getPort());

    return err;
}

esp_err_t DACx0501::set(int value) {
    esp_err_t err = ESP_OK;

    uint32_t dac_value = value * 65536 / 2500;
    uint8_t dac_cmd_set[3] = { DACx0501::m_cmd_dacdata, (uint8_t) ((dac_value >> 8) & 0xFF),
            (uint8_t) (dac_value & (0xFF - (1 << (16 - this->m_bits)) + 1)) };

    ESP_LOGD(LOG_TAG, "CMD: 0x%02X %02X %02X", dac_cmd_set[0], dac_cmd_set[1], dac_cmd_set[2]);

    I2CDescriptor *i2c_desc = static_cast<I2CDescriptor*>(m_hardware_descriptor);
    HardwareDescriptor::lockI2C(i2c_desc->getPort());
    err = i2c_desc->init();

    err = i2c_desc->write(dac_cmd_set, 3);

    i2c_desc->deinit();
    HardwareDescriptor::unlockI2C(i2c_desc->getPort());

    ESP_LOGV(LOG_TAG, "Set error:%s", esp_err_to_name(err));
    return err;
}

esp_err_t DACx0501::enable() {
    esp_err_t err = ESP_OK;

    uint8_t dac_cmd[3] = { DACx0501::m_cmd_config, 0x00, 0x00 };

    I2CDescriptor *i2c_desc = static_cast<I2CDescriptor*>(m_hardware_descriptor);
    HardwareDescriptor::lockI2C(i2c_desc->getPort());
    err = i2c_desc->init();

    if (err == ESP_OK) {
        err = i2c_desc->write(dac_cmd, 3);
    }

    i2c_desc->deinit();
    HardwareDescriptor::unlockI2C(i2c_desc->getPort());

    return err;
}

esp_err_t DACx0501::disable() {
    esp_err_t err = ESP_OK;

    uint8_t dac_cmd[3] = { DACx0501::m_cmd_config, 0x01, 0x01 };

    I2CDescriptor *i2c_desc = static_cast<I2CDescriptor*>(m_hardware_descriptor);
    HardwareDescriptor::lockI2C(i2c_desc->getPort());
    err = i2c_desc->init();

    if (err == ESP_OK) {
        err = i2c_desc->write(dac_cmd, 3);
    }

    i2c_desc->deinit();
    HardwareDescriptor::unlockI2C(i2c_desc->getPort());


    return err;
}

uint8_t DACx0501::getBits() {
    return m_bits;
}
