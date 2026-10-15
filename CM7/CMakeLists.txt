cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Compiler options
set(STM32_MCU_FLAGS  "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard " )

#Linker options
set(STM32_LINKER_SCRIPT stm32h755xx_flash_CM7.ld)
set(STM32_LINKER_OPTION  )

# Set the project name
set(CMAKE_PROJECT_NAME sfurs_firmware_v2_CM7)
project(${CMAKE_PROJECT_NAME})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Include mx-generated
include("mx-generated.cmake")


# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common/Inc
)

# Add sources to executable
option(NUCLEO_MOTOR_ENABLED "Enable explicit wheel bench ARM/RUN commands" OFF)
option(NUCLEO_IK_ENABLED "Exclusive keyboard IK mode (no bench/CAL/FWU pump)" OFF)
if(NUCLEO_IK_ENABLED)
    if(NOT NUCLEO_MOTOR_ENABLED)
        message(FATAL_ERROR "IK requires NUCLEO_MOTOR_ENABLED=ON")
    endif()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE NUCLEO_IK_ENABLED=1)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Src/ik_drive.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../Common/Src/rover_kinematics.c
    )
endif()
if(NUCLEO_MOTOR_ENABLED)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FWU_BLINK_BENCH_ONLY=0U)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FWU_BLINK_BENCH_ONLY=1U)
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(FWU_IMAGE_FILE "" CACHE FILEPATH "Generated FWU payload C file (empty selects blink test)")
if(FWU_IMAGE_FILE)
    get_filename_component(FWU_PAYLOAD "${FWU_IMAGE_FILE}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if(NOT EXISTS "${FWU_PAYLOAD}")
        message(FATAL_ERROR "FWU payload does not exist: ${FWU_PAYLOAD}")
    endif()
else()
set(FWU_BLINK_DIR "${CMAKE_CURRENT_BINARY_DIR}/esc-blink")
add_custom_command(
    OUTPUT "${FWU_BLINK_DIR}/fwu_blink_image.c"
    COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/../tests/esc_blink/build.py"
        --cc "${CMAKE_C_COMPILER}" --output "${FWU_BLINK_DIR}"
    DEPENDS ../tests/esc_blink/build.py ../tests/esc_blink/main.c
        ../tests/esc_blink/app.ld ../tests/esc_blink/can_boot.c
        ../tests/esc_blink/can_boot.h ../Common/Inc/boot_entry.h
    VERBATIM
)
set(FWU_PAYLOAD "${FWU_BLINK_DIR}/fwu_blink_image.c")
endif()
message(STATUS "FWU payload: ${FWU_PAYLOAD}")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/wheel_bench.c
    Core/Src/can_cal_sender.c
    Core/Src/can_fwu_host.c
    "${FWU_PAYLOAD}"
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common/Src/can_fwu_protocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common/Src/can_fwu_boot.c
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined libraries
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()



