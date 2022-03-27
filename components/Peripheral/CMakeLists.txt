idf_component_register(SRCS "DS18B20_UART.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver hal esp_common ThreadSafeDriver 
                    PRIV_REQUIRES )

