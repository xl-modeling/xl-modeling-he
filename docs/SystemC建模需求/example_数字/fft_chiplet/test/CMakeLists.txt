cmake_minimum_required(VERSION 3.1)
project(test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Dependency 
set(SYSTEMC_HOME $ENV{SYSTEMC_HOME} CACHE PATH "Path to SystemC Home")
set(SYSTEMC_AMS_HOME $ENV{SYSTEMC_AMS_HOME} CACHE  PATH "Path to SystemC-AMS Home")

if(SYSTEMC_HOME)
    list(APPEND CMAKE_PREFIX_PATH ${SYSTEMC_HOME})
endif()

find_package(SystemCLanguage CONFIG REQUIRED)
find_package(SystemCTLM CONFIG REQUIRED)



# SystemC-AMS-2.3.4 没有对应的Config.cmake文件，需要自行寻找
if(SYSTEMC_AMS_HOME)

else()
    
endif()

add_compile_options(-g -Wall -Wextra -Werror)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install OFF CACHE INTERNAL "")

add_executable(fft_chiplet ${SRC} )

find_package(nlohmann_json 3.12.0 REQUIRED)

target_link_libraries(fft_chiplet SystemC::systemc)
target_link_libraries(fft_chiplet Boost::boost)
target_link_libraries(fft_chiplet Boost::filesystem)
target_link_libraries(fft_chiplet nlohmann_json::nlohmann_json)