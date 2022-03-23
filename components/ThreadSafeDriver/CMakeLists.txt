idf_component_register(SRCS "UARTThreadSafeDriver.cpp" "I2CThreadSafeDriver.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_common esp_adc_cal hal)

