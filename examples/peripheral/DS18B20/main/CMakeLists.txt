#idf_component_register(SRCS "ds18b20_example_main.cpp"
#                    INCLUDE_DIRS "." 
#                    "$ENV{CONNAXIO_IDF_PATH}/components/Peripheral/include/"
#                    "$ENV{CONNAXIO_IDF_PATH}/components/ThreadSafeDriver/include/")          
                    
set(COMPONENT_REQUIRES ThreadSafeDriver Peripheral)
set(COMPONENT_PRIV_REQUIRES hal driver esp_common freertos)

set(COMPONENT_SRCS "ds18b20_example_main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "$ENV{CONNAXIO_IDF_PATH}/components/Peripheral/include/"
                    "$ENV{CONNAXIO_IDF_PATH}/components/ThreadSafeDriver/include/")

register_component()
                    