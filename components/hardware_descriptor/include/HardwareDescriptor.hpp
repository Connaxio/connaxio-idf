#ifndef _HARDWARE_DESCRIPTOR_H_
#define _HARDWARE_DESCRIPTOR_H_


#include <stdint.h>

#include "driver/uart.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

class HardwareDescriptor {
public:
	HardwareDescriptor();
	virtual ~HardwareDescriptor();

	static void lockUART(uint8_t uart_num);
	static void lockI2C(uint8_t i2c_num);
	static void unlockUART(uint8_t uart_num);
	static void unlockI2C(uint8_t i2c_num);

protected:


private:

	static SemaphoreHandle_t m_hardware_descriptor_counter_semaphore;
	static SemaphoreHandle_t m_UARTMutexes[UART_NUM_MAX];
	static SemaphoreHandle_t m_I2CMutexes[I2C_NUM_MAX];

	static void initMutexes();
	static void deinitMutexes();
};

#endif /* _HARDWARE_DESCRIPTOR_H_ */
