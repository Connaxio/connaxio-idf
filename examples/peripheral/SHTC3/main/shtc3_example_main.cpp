#include <stdio.h>

#include "SHTC3.hpp"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
void app_main(void);
}

void app_main(void) {
	SHTC3 *shtc3 = new SHTC3(GPIO_NUM_23, GPIO_NUM_18);

	while (true) {
		shtc3->update();
		printf("Temperature:\t%.02f C.\r\n", shtc3->getTemperature_celsius());
		printf("Humidity:\t%.02f %%.\r\n", shtc3->getHumidity_pct());
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	// Should not reach here
	delete shtc3;
}
