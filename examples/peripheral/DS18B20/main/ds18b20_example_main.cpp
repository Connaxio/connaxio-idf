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

void app_main(void) {
	// Create a OneWireUARTThreadSafeDriver to explore the bus devices.
	OneWireUARTThreadSafeDriver *ow_driver = new OneWireUARTThreadSafeDriver(GPIO_NUM_10, GPIO_NUM_9);
	std::vector<std::array<uint8_t, 8>> addresses;
	// Use the driver to search the bus and find all 1-wire devices.
	ow_driver->searchBus(addresses);
	// Find the first DS18B20 on the bus (2nd parameter).
	std::array<uint8_t, 8> address = ow_driver->findFamilyDevice(DS18B20_FAMILY_ID, 0, addresses);
	// Delete the initial driver, we no longer need it.
	delete (ow_driver);

	// Abort if no device was found.
	if (address.at(0) != DS18B20_FAMILY_ID) {
		ESP_LOGI("MAIN", "ERROR: No DS18B20 found on bus.");
		abort();
	}

	uint8_t resolution_bits = 11;
	DS18B20_UART *ds18b20 = new DS18B20_UART(GPIO_NUM_10, GPIO_NUM_9, address, resolution_bits);
	ds18b20->initialize();

	while (true) {
		ds18b20->update();
		printf("Temperature:\t%.02f C.\r\n", ds18b20->getTemperature_celsius());

		ds18b20->startConversion();
		vTaskDelay(pdMS_TO_TICKS(752 >> (12 - resolution_bits)));
		printf("Temperature:\t%.02f C.\r\n", ds18b20->getTemperature_celsius());

		ds18b20->startConversionAll();
		vTaskDelay(pdMS_TO_TICKS(752 >> (12 - resolution_bits)));
		printf("Temperature:\t%.02f C.\r\n", ds18b20->getTemperature_celsius());
	}
	// Should not reach here
	delete ds18b20;
}

