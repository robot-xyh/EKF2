cmake_minimum_required(VERSION 3.16)

project(OrangePi5_EKF2 C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_custom_target(prebuild_targets)

add_compile_definitions(ECL_STANDALONE)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/Matrix)
add_subdirectory(external/PX4-ECL EXCLUDE_FROM_ALL)

set(COMMON_C
    src/bsp/bsp.c
    src/bsp/gpio.c
    src/bsp/i2c.c
    src/bsp/spi.c
    src/bsp/uart.c
    src/drivers/dps310.c
    src/drivers/icm40608.c
    src/drivers/qmc5883p.c
    src/drivers/ubx_gps.c
    src/app/ahrs.c
    src/app/altitude.c
)

add_executable(sensor_ekf2
    src/ekf2/sensor_ekf2.c
    src/ekf2/ekf2_nav.cpp
    ${COMMON_C}
)

target_include_directories(sensor_ekf2 PRIVATE
    src/app
    src/bsp
    src/drivers
    src/ekf2
    external/PX4-ECL
    external/Matrix
)

target_compile_options(sensor_ekf2 PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
)

target_link_libraries(sensor_ekf2 PRIVATE ecl_EKF m)
