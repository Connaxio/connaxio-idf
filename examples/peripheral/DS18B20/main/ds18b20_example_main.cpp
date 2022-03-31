/*
 * ds18b20_example_main.cpp
 *
 *  Created on: Mar. 23, 2022
 *      Author: malalonde
 */

#include <stdio.h>
#include <vector>

#include "DS18B20_UART.hpp"
#include "esp_log.h"

extern "C" {
void app_main(void);
}

// Specific confiuration for Espoir IO One HAT Rev 0.3.3
void configureIOs() {

	// Enable power for the target connector
	gpio_config_t gpio_conf = {
			.pin_bit_mask = (1ULL << GPIO_NUM_5),
			.mode = GPIO_MODE_OUTPUT_OD,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE };
	gpio_config(&gpio_conf);
	gpio_set_level(GPIO_NUM_5, 0);
}

void app_main(void) {
	configureIOs();

	// Create a OneWireUARTThreadSafeDriver to explore the bus devices.
	OneWireUARTThreadSafeDriver *ow_driver = new OneWireUARTThreadSafeDriver(GPIO_NUM_9, GPIO_NUM_10);
	std::vector<std::array<uint8_t, 8>> addresses;
	// Use the driver to search the bus.
	ow_driver->searchBus(addresses);
	// Find the first DS18B20 on the bus.
	std::array<uint8_t, 8> address = ow_driver->findFamilyDevice(DS18B20_FAMILY_ID, 0, addresses);
	// Delete the initial driver, we no longer need it.
	delete (ow_driver);

	// Abort if no device was found.
	if (address.at(0) != DS18B20_FAMILY_ID) {
		ESP_LOGI("MAIN", "ERROR: No DS18B20 found on bus.");
		abort();
	}

	uint8_t resolution_bits = 11;
	DS18B20_UART *ds18b20 = new DS18B20_UART(GPIO_NUM_9, GPIO_NUM_10, address, resolution_bits);
	ds18b20->initialize();

	while (true) {
		ds18b20->update();
		printf("Temperature:\t%.02f degC.\r\n", ds18b20->getTemperature_celsius());

		ds18b20->startConversion();
		vTaskDelay(pdMS_TO_TICKS(752 >> (12 - resolution_bits)));
		printf("Temperature:\t%.02f degC.\r\n", ds18b20->getTemperature_celsius());

		ds18b20->startConversionAll();
		vTaskDelay(pdMS_TO_TICKS(752 >> (12 - resolution_bits)));
		printf("Temperature:\t%.02f degC.\r\n", ds18b20->getTemperature_celsius());
	}
	// Should not reach here
	delete (ds18b20);
}

