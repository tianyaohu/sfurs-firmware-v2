cmake_minimum_required(VERSION 3.22)

# Host cross-compile of the vendored B-G431B-ESC1 CubeIDE project.
# CubeIDE does not ship CMake. STM32_MCU_FLAGS is read by
# src/nucleo/gcc-arm-none-eabi.cmake when project() loads the toolchain.

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# STM32G431CBUx on the B-G431B-ESC1 (Cortex-M4F).
set(STM32_MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(STM32_LINKER_SCRIPT "STM32CubeIDE/STM32G431CBUX_FLASH.ld")
set(STM32_LINKER_OPTION "")

set(CMAKE_CXX_COMPILER_FORCED TRUE)

set(CMAKE_PROJECT_NAME motor_config)
project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32G431xx
    ARM_MATH_CM4
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Inc
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Generated project (Cube startup and MCSDK sources under Src/)
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32CubeIDE/Application/Startup/startup_stm32g431cbux.s
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32CubeIDE/Application/User/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32CubeIDE/Application/User/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/aspep.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/hall_speed_pos_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/hf_registers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_api.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_app_hooks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_config_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_configuration_registers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_interface.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_parameters.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mc_tasks_foc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mcp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mcp_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/motorcontrol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/pwm_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/pwm_curr_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/regular_conversion_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/speed_torq_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32_mc_common_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32g4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32g4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32g4xx_mc_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/sync_registers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/system_stm32g4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/usart_aspep_driver.c
    # HAL
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_comp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cordic.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    # MCSDK
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/bus_voltage_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/circle_limitation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/digital_output.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/mcpa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/ntc_temperature_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/open_loop.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/pid_regulator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/pqd_motor_power_measurement.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/r_divider_bus_voltage_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/ramp_ext_mngr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/revup_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/speed_pos_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/virtual_speed_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE m)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
