/*
 * DS18B20.cpp
 *
 *  Created on: Aug 30, 2021
 *      Author: Marc-Antoine
 */

#include "DS18B20.hpp"

#include <math.h>

#include "OneWireUARTDescriptor.hpp"

#define LOG_TAG "DS18B20"

#define TOPIC_TEMPERATURE "temperature"

#define DS18B20_FAMILY_ID 0x28

// DS18B20 ROM Commands
#define DS18B20_SEARCH_ROM 0xf0
#define DS18B20_READ_ROM 0x33
#define DS18B20_MATCH_ROM 0x55
#define DS18B20_SKIP_ROM 0xcc
#define DS18B20_ALARM_SEARCH 0xec

// DS18B20 Function Commands
#define DS18B20_START_CONVERSION 0x44
#define DS18B20_READ_SCRATCHPAD 0xbe
#define DS18B20_WRITE_SCRACHPAD 0x4e
#define DS18B20_COPY_SCRACHPAD 0x48
#define DS18B20_RECALL_E2 0xb8
#define DS18B20_READ_POWER_SUPPLY 0xb4

DS18B20::DS18B20(std::string t_name, gpio_num_t t_tx_gpio_num, gpio_num_t t_rx_gpio_num,
		gpio_num_t t_enable_pin, uint16_t t_turn_on_time_ms, uint8_t t_enable_level) :
		Peripheral(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level),
		Sensor(t_name, t_enable_pin, t_turn_on_time_ms, t_enable_level) {

	m_values[TOPIC_TEMPERATURE] = NAN;
	m_hardware_descriptor = new OneWireUARTDescriptor(t_tx_gpio_num, t_rx_gpio_num);

	this->init();

}

DS18B20::~DS18B20() {
	delete (m_hardware_descriptor);
}

esp_err_t DS18B20::init() {
	this->turnONAndWait();
	HardwareDescriptor::lockUART(OneWireUARTDescriptor::getUARTNum());
	esp_err_t err = static_cast<OneWireUARTDescriptor*>(m_hardware_descriptor)->findDevice(
			(uint8_t) DS18B20_FAMILY_ID);
	HardwareDescriptor::unlockUART(OneWireUARTDescriptor::getUARTNum());
	this->turnOFF();

	return err;
}

esp_err_t DS18B20::update() {
	esp_err_t err = ESP_OK;
	int16_t temp = 126 << 4;
	OneWireUARTDescriptor *t_ow_desc = static_cast<OneWireUARTDescriptor*>(m_hardware_descriptor);

	if (t_ow_desc->hasValidROM()) {

		this->turnONAndWait();
		HardwareDescriptor::lockUART(OneWireUARTDescriptor::getUARTNum());

		// Set the DS18B20 to measure with 11 bits
		if (t_ow_desc->matchROM() == ESP_OK) {
			t_ow_desc->writeByte(0);
			t_ow_desc->writeByte(0);
			t_ow_desc->writeByte(0b01011111);
			vTaskDelay(pdMS_TO_TICKS(1) + 1);

			// Match the id found earlier.
			t_ow_desc->matchROM();
			// Send start conversion command.
			t_ow_desc->writeByte(DS18B20_START_CONVERSION);
			// Wait until conversion is finished.
			HardwareDescriptor::unlockUART(OneWireUARTDescriptor::getUARTNum());

			// Bus line is held low until conversion is finished.
			// UART can be used with other pins, but this bus is occupied for the duration of the conversion.
			uint8_t done = 0;
			for (uint8_t i = 0; i < 10 && !done; ++i) {
				vTaskDelay(pdMS_TO_TICKS(50));
				HardwareDescriptor::lockUART(OneWireUARTDescriptor::getUARTNum());
				done = t_ow_desc->readBit();
				HardwareDescriptor::unlockUART(OneWireUARTDescriptor::getUARTNum());
			}

			HardwareDescriptor::lockUART(OneWireUARTDescriptor::getUARTNum());
			// Match id again.
			t_ow_desc->matchROM();
			// Send READ SCRATCHPAD command.
			t_ow_desc->writeByte(DS18B20_READ_SCRATCHPAD);
			// Read only two first bytes (temperature low, temperature high)
			// and place them in the 16 bit temperature variable.
			temp = t_ow_desc->readByte();
			temp |= (t_ow_desc->readByte() << 8);
		} else {
			t_ow_desc->resetROM();
		}

		HardwareDescriptor::unlockUART(OneWireUARTDescriptor::getUARTNum());
		this->turnOFF();

		/* Validate reading. Notation Q12.4 */
		if (((temp >> 4) > -55) && ((temp >> 4) < 125)) {
			m_values[TOPIC_TEMPERATURE] = ((float) temp) * 0.0625; // Equivalent of ( >> 4 ) but with decimals
			err = ESP_OK;
		} else {
			m_values[TOPIC_TEMPERATURE] = NAN;
			err = ESP_FAIL;
		}
	} else {
		err = this->init();
		if (err == ESP_OK) {
			this->update();
		}
	}

	return err;
}
