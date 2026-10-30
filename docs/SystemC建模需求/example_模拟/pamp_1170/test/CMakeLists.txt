cmake_minimum_required(VERSION 3.1)
project(pamp_1170)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 17)

# SystemC / SystemC-AMS 通过环境变量定位（无 CMake config 文件）
set(CMAKE_PREFIX_PATH $ENV{SYSTEMC_HOME};$ENV{SYSTEMC_AMS_HOME})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 编译选项
add_compile_options(-O0 -g -Wall -Wextra -Wunused-function -Wpedantic)
add_compile_options(-Wno-unused-parameter -Wno-unused-variable
                    -Wno-unused-but-set-variable -Wno-unused-function)
add_compile_options(-Wno-reorder)

# 头文件搜索路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/)
include_directories($ENV{SYSTEMC_HOME}/include)
include_directories($ENV{SYSTEMC_AMS_HOME}/include)

# 库搜索路径
link_directories($ENV{SYSTEMC_HOME}/lib-linux64)
link_directories($ENV{SYSTEMC_AMS_HOME}/lib-linux64)

# nlohmann_json 的编译选项
set(JSON BuildTestS OFF CACHE INTERNAL "")
set(JSON InstaLL OFF CACHE INTERNAL "")

# 源文件：source/*.cpp + test/*.cpp
file(GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/../source/*.cpp"
              "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_executable(pamp_1170 ${SRC})

# 依赖与链接
find_package(nlohmann_json 3.12.0 REQUIRED)
target_link_libraries(pamp_1170 systemc systemc-ams)
target_link_libraries(pamp_1170 nlohmann_json::nlohmann_json)
target_link_libraries(pamp_1170 m)

# gold 链接器
add_link_options(-B/usr/lib/gold-ld/)
