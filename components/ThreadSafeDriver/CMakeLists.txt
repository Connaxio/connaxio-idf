idf_component_register(SRCS "UARTThreadSafeDriver.cpp" "I2CThreadSafeDriver.cpp" "OneWireUARTThreadSafeDriver.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_common freertos hal
                    PRIV_REQUIRES )

