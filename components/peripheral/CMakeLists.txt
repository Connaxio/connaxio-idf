idf_component_register(SRCS "Peripheral.cpp" "Sensor.cpp" "CSMS12.cpp" "DS18B20.cpp" "SHTC3.cpp" "Servo.cpp" 
							"SensorSwitch.cpp" "ActuatorSwitch.cpp" "ADS1119.cpp" "DACx0501.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver hardware_descriptor esp_common)
