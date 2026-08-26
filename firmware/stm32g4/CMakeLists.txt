cmake_minimum_required(VERSION 3.20)
project(apexdrive_stm32g4_firmware C CXX ASM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Microcontroller Target Configuration (ARM Cortex-M4F)
set(MCU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -Wall -Wextra -Werror -O3 -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCU_FLAGS} -Wall -Wextra -Werror -O3 -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections")

include_directories(
    include
    ../../include
)

add_executable(apexdrive_firmware.elf
    src/main.cpp
)

target_link_libraries(apexdrive_firmware.elf
    -Wl,--gc-sections
)
