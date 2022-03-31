/*
 * HardwareDescriptor.cpp
 *
 *  Created on: Sep 7, 2021
 *      Author: Marc-Antoine
 */

#include "HardwareDescriptor.hpp"

SemaphoreHandle_t HardwareDescriptor::m_hardware_descriptor_counter_semaphore = xSemaphoreCreateCounting(0xFFFFFFFF, 0);
SemaphoreHandle_t HardwareDescriptor::m_UARTMutexes[UART_NUM_MAX];
SemaphoreHandle_t HardwareDescriptor::m_I2CMutexes[I2C_NUM_MAX];

HardwareDescriptor::HardwareDescriptor() {
	xSemaphoreGive(HardwareDescriptor::m_hardware_descriptor_counter_semaphore);
	if (uxSemaphoreGetCount(HardwareDescriptor::m_hardware_descriptor_counter_semaphore) == 1) {
		HardwareDescriptor::initMutexes();
	}
}

HardwareDescriptor::~HardwareDescriptor() {
	xSemaphoreTake(HardwareDescriptor::m_hardware_descriptor_counter_semaphore, 0);
	if (uxSemaphoreGetCount(HardwareDescriptor::m_hardware_descriptor_counter_semaphore) == 0) {
			HardwareDescriptor::deinitMutexes();
		}
}

void HardwareDescriptor::initMutexes() {
	for (uint8_t i = 0; i < UART_NUM_MAX; ++i) {
		HardwareDescriptor::m_UARTMutexes[i] = xSemaphoreCreateMutex();
	}

	for (uint8_t i = 0; i < I2C_NUM_MAX; ++i) {
		HardwareDescriptor::m_I2CMutexes[i] = xSemaphoreCreateMutex();
	}
}

void HardwareDescriptor::deinitMutexes() {
	for (uint8_t i = 0; i < UART_NUM_MAX; ++i) {
		vSemaphoreDelete(HardwareDescriptor::m_UARTMutexes[i]);
	}

	for (uint8_t i = 0; i < I2C_NUM_MAX; ++i) {
		vSemaphoreDelete(HardwareDescriptor::m_I2CMutexes[i]);
	}
}

void HardwareDescriptor::lockUART(uint8_t t_uart_num) {
	xSemaphoreTake(HardwareDescriptor::m_UARTMutexes[t_uart_num], portMAX_DELAY);

}
void HardwareDescriptor::lockI2C(uint8_t t_i2c_num) {
	xSemaphoreTake(HardwareDescriptor::m_I2CMutexes[t_i2c_num], portMAX_DELAY);
}

void HardwareDescriptor::unlockUART(uint8_t t_uart_num) {
	xSemaphoreGive(HardwareDescriptor::m_UARTMutexes[t_uart_num]);

}
void HardwareDescriptor::unlockI2C(uint8_t t_i2c_num) {
	xSemaphoreGive(HardwareDescriptor::m_I2CMutexes[t_i2c_num]);
}
