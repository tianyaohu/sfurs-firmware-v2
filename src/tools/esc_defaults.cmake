# Generates esc_defaults.h from the ESC's drive_parameters.h so the PC tools restore
# exactly the values the firmware boots with and check the same limits it enforces
# (ESC_MAX_APPLICATION_SPEED_RPM, ESC_IQMAX_A in amperes). drive_parameters.h pulls in HAL types,
# so it cannot be included directly; its plain `#define NAME number` lines are copied.
#
#   include(src/tools/esc_defaults.cmake)
#   generate_esc_defaults(${CMAKE_BINARY_DIR}/gen/esc_defaults.h)
set(ESC_DEFAULT_NAMES
  PID_SPEED_KP_DEFAULT PID_SPEED_KI_DEFAULT
  SPEED_STALL_RAMP_MA_PER_S_DEFAULT SPEED_STALL_MS_DEFAULT
  MAX_APPLICATION_SPEED_RPM IQMAX_A)

function(generate_esc_defaults out)
  set(params ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../esc/motor_config/Inc/drive_parameters.h)
  file(READ ${params} text)
  set(body "/* Generated from drive_parameters.h by esc_defaults.cmake. Do not edit. */\n#ifndef ESC_DEFAULTS_H\n#define ESC_DEFAULTS_H\n")
  foreach(name ${ESC_DEFAULT_NAMES})
    if(NOT text MATCHES "#define ${name}[ \t]+([0-9]+)")
      message(FATAL_ERROR "drive_parameters.h: no numeric #define ${name}")
    endif()
    string(APPEND body "#define ESC_${name} ${CMAKE_MATCH_1}\n")
  endforeach()
  string(APPEND body "#endif\n")
  file(WRITE ${out} "${body}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${params})
endfunction()
