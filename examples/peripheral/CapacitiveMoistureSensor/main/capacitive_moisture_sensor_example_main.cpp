#include <stdio.h>

#include "CapacitiveMoistureSensor.hpp"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
void app_main(void);
}

// Specific configuration for Espoir IO One HAT Rev 0.3.3
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

	gpio_conf.mode = GPIO_MODE_DISABLE;
	gpio_conf.pin_bit_mask = (1ULL << GPIO_NUM_10);
	gpio_config(&gpio_conf);

	gpio_conf.pin_bit_mask = (1ULL << GPIO_NUM_9);
	gpio_config(&gpio_conf);
}

void app_main(void) {
	// Enable power for the target connector
	configureIOs();

	CapacitiveMoistureSensor *moisture_sensor = new CapacitiveMoistureSensor(GPIO_NUM_36);

	while (true) {
		moisture_sensor->update();
		printf("Moisture:\t%.02f %%.\r\n", moisture_sensor->getMoisture_pct());
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	// Should not reach here
	delete moisture_sensor;
}
