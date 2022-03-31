idf_component_register(SRCS "DS18B20_UART.cpp" "CapacitiveMoistureSensor.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver hal esp_common ThreadSafeDriver esp_adc_cal
                    PRIV_REQUIRES )

