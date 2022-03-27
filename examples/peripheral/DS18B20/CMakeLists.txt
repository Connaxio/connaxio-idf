# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(ENV{CONNAXIO_IDF_PATH} "/media/veracrypt1/Git/code/connaxio-idf")
include($ENV{CONNAXIO_IDF_PATH}/component.cmake)
#list(APPEND EXTRA_COMPONENTS_DIR "$ENV{CONNAXIO_IDF_PATH}/components/ThreadSafeDriver")
#list(APPEND EXTRA_COMPONENTS_DIR "$ENV{CONNAXIO_IDF_PATH}/components/Peripheral")




include($ENV{IDF_PATH}/tools/cmake/project.cmake)
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})
