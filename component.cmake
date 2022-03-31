add_compile_options(-fdiagnostics-color=always)

list(APPEND EXTRA_COMPONENT_DIRS 
                                "$ENV{CONNAXIO_IDF_PATH}/components"
                                "$ENV{CONNAXIO_IDF_PATH}/components/deprecated"
                                )