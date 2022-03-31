idf_component_register(SRCS "HardwareDescriptor.cpp" "ADCDescriptor.cpp" "I2CDescriptor.cpp" "OneWireUARTDescriptor.cpp" "PWMDescriptor.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_common esp_adc_cal)
