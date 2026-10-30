cmake_minimum_required(VERSION 3.1)
project(test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Dependency 
set(SYSTEMC_HOME $ENV{SYSTEMC_HOME} CACHE PATH "Path to SystemC Home")
set(SYSTEMC_AMS_HOME $ENV{SYSTEMC_AMS_HOME} CACHE PATH "Path to SystemC-AMS Home")

if(SYSTEMC_HOME)
    list(APPEND CMAKE_PREFIX_PATH ${SYSTEMC_HOME})
endif()

find_package(SystemCLanguage CONFIG REQUIRED)
find_package(SystemCTLM CONFIG REQUIRED)

# SystemC-AMS-2.3.4 没有对应的Config.cmake文件，需要自行寻找
find_path(SYSTEMC_AMS_INCLUDE_DIR
    NAMES systemc-ams systemc-ams.h
    HINTS ${SYSTEMC_AMS_HOME}/include
)

find_library(SYSTEMC_AMS_LIBRARIES
    NAMES systemc-ams
    HINTS ${SYSTEMC_AMS_HOME}/lib ${SYSTEMC_AMS_HOME}/lib-linux64
)

if(SystemCLanguage_FOUND)
    message(STATUS "SystemC found: ${SystemCLanguage_DIR}")
else()
    message(FATAL_ERROR "SystemC not found")
endif()

if(SYSTEMC_AMS_INCLUDE_DIR AND SYSTEMC_AMS_LIBRARIES)
    get_filename_component(SYSTEMC_AMS_HOME "${SYSTEMC_AMS_INCLUDE_DIR}/.." ABSOLUTE)
    message(STATUS "SystemC-AMS found: ${SYSTEMC_AMS_HOME}")
else()
    message(FATAL_ERROR "SystemC-AMS not found")
endif()

# Compile and Link
add_compile_options(-g -Wall -Werror)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} TESTBENCH)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../source SRC)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include ${SystemCLanguage_INCLUDE_DIR} ${SYSTEMC_AMS_INCLUDE_DIR})

# gold 链接器
add_link_options(-fuse-ld=gold)

foreach(tb IN LISTS TESTBENCH)
    get_filename_component(tb_name "${tb}" NAME_WE)
    add_executable(${tb_name} ${tb} ${SRC})
    target_link_libraries(${tb_name} PRIVATE
        SystemC::systemc
        ${SYSTEMC_AMS_LIBRARIES}
    )
endforeach()