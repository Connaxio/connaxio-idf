/*
 * ADS1119IPWR.cpp
 *
 *  Created on: Jan. 2, 2022
 *      Author: malalonde
 */

#include <math.h>

#include "ADS1119.hpp"
#include "esp_log.h"

#include "I2CDescriptor.hpp"

#define LOG_TAG "ADS1119"

#define ADS1119_I2C_NUM       I2C_NUM_0

#define CONFIG  0b00001100

#define TOPIC_CH0   "ch0"
#define TOPIC_CH1   "ch1"
#define TOPIC_CH2   "ch2"
#define TOPIC_CH3   "ch3"

const uint8_t ADS1119::m_cmd_reset = 0x06;
const uint8_t ADS1119::m_cmd_start = 0x08;
const uint8_t ADS1119::m_cmd_pwrdn = 0x02;
const uint8_t ADS1119::m_cmd_rdata = 0x10;
const uint8_t ADS1119::m_cmd_rreg_conf = 0x20;
const uint8_t ADS1119::m_cmd_rreg_status = 0x24;
const uint8_t ADS1119::m_cmd_wreg = 0x40;

ADS1119::ADS1119(std::string t_name, gpio_num_t t_sda_gpio_num, gpio_num_t t_scl_gpio_num,
        gpio_num_t t_enable_pin, uint16_t t_turn_on_time_ms, uint8_t t_enable_level) :
        Peripheral(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level),
        Sensor(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level) {
    m_offset = 0;
    m_values[TOPIC_CH0] = NAN;
    m_values[TOPIC_CH1] = NAN;
    m_values[TOPIC_CH2] = NAN;
    m_values[TOPIC_CH3] = NAN;
    m_hardware_descriptor = new I2CDescriptor(t_sda_gpio_num, t_scl_gpio_num, 400000, ADS1119_I2C_NUM,
            m_address);

}

ADS1119::~ADS1119() {
    // TODO Auto-generated destructor stub
}

esp_err_t ADS1119::init() {
    esp_err_t err = ESP_OK;
    uint8_t config_cmd[2] = { ADS1119::m_cmd_wreg, (0b111 << 5) | CONFIG };
    uint8_t readout[2] = { 0, 0 };
    uint32_t offset_sum = 0;

    I2CDescriptor *i2c_desc = static_cast<I2CDescriptor*>(m_hardware_descriptor);
    HardwareDescriptor::lockI2C(i2c_desc->getPort());
    err = i2c_desc->init();

    ESP_LOGV(LOG_TAG, "Initializing ADS1119IPWR");
    /* Reset device */
    if (err == ESP_OK) {
        err = i2c_desc->write(&ADS1119::m_cmd_reset, 1);
        ESP_LOGV(LOG_TAG, "Reset command returned: %s", esp_err_to_name(err));

        /* Write config to device */
        if (err == ESP_OK) {
            err = i2c_desc->write(config_cmd, 2);

            if (err == ESP_OK) {
                err = i2c_desc->write(&ADS1119::m_cmd_rreg_conf, 1);
                if (err == ESP_OK) {
                    err = i2c_desc->read(&readout[0], 1);
                    if (err == ESP_OK) {
                        err = readout[0] == config_cmd[1] ? ESP_OK : ESP_FAIL;
                    }
                }
            }
        }
    }

    i2c_desc->deinit();
    HardwareDescriptor::unlockI2C(i2c_desc->getPort());

    if (err == ESP_OK) {
        m_offset = offset_sum >> 4;
        ESP_LOGD(LOG_TAG, "Offset: %i", m_offset);
    }

    return err;
}

esp_err_t ADS1119::update() {
    esp_err_t err = ESP_OK;

    for (uint8_t i = 0; i < 4 && err == ESP_OK; ++i) {
        updateChannel(i);
    }

    return err;
}

esp_err_t ADS1119::updateChannel(uint8_t channel) {
    esp_err_t err = ESP_OK;
    uint8_t cmd_conf[2] = { ADS1119::m_cmd_wreg, CONFIG };
    uint8_t readout[2] = { 0, 0 };
    /* Select correct channel to read */
    if (channel == 0) {
        cmd_conf[1] |= (0b011 << 5);
    } else if (channel == 1) {
        cmd_conf[1] |= (0b100 << 5);
    } else if (channel == 2) {
        cmd_conf[1] |= (0b101 << 5);
    } else if (channel == 3) {
        cmd_conf[1] |= (0b110 << 5);
    }

    /* Create and initialize I2C descriptor */
    I2CDescriptor *i2c_desc = static_cast<I2CDescriptor*>(m_hardware_descriptor);

    HardwareDescriptor::lockI2C(i2c_desc->getPort());
    err = i2c_desc->init();

    if (err == ESP_OK) {
        /* Write config to device */
        err = i2c_desc->write(cmd_conf, 2);
        if (err == ESP_OK) {
            /* Request ADC read */
            err = i2c_desc->write(&ADS1119::m_cmd_start, 1);
            if (err == ESP_OK) {
                /* Wait 1ms */
                vTaskDelay(pdMS_TO_TICKS(1));

                /* Read status register until data is ready */
                for (uint8_t j = 0; j < 50 && (readout[0] & 0x80) == 0; ++j) {
                	taskYIELD();
                    if (err == ESP_OK) {
                        err = i2c_desc->write(&ADS1119::m_cmd_rreg_status, 1);

                        if (err == ESP_OK) {
                            err = i2c_desc->read(readout, 1);
                        }
                    }
                }

                /* Verify that there was valid data to read */
                if ((readout[0] & 0x80) == 0) {
                    err = ESP_ERR_NOT_FINISHED;
                }
                /* Read data */
                if (err == ESP_OK) {
                    err = i2c_desc->write(&ADS1119::m_cmd_rdata, 1);

                    if (err == ESP_OK) {
                        err = i2c_desc->read(readout, 2);

                    }
                }
            }
        }
    }
    /* Destroy I2C descriptor */
    i2c_desc->deinit();
    HardwareDescriptor::unlockI2C(i2c_desc->getPort());

    /* Update correct value field */
    if (err == ESP_OK) {
        int16_t value = (readout[0] << 8) + readout[1];
        float voltage = value * 2048.0f / 32768.0f;

        if (channel == 0) {
            m_values[TOPIC_CH0] = voltage;
        } else if (channel == 1) {
            m_values[TOPIC_CH1] = voltage;
        } else if (channel == 2) {
            m_values[TOPIC_CH2] = voltage;
        } else if (channel == 3) {
            m_values[TOPIC_CH3] = voltage;
        }
    }

    return err;
}
