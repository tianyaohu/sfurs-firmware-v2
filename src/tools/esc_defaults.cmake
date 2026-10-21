# Generates esc_defaults.h from the ESC's drive_parameters.h so the PC tools restore
# exactly the values the firmware boots with. drive_parameters.h pulls in HAL types,
# so it cannot be included directly; its plain `#define NAME number` lines are copied.
#
#   include(src/tools/esc_defaults.cmake)
#   generate_esc_defaults(${CMAKE_BINARY_DIR}/gen/esc_defaults.h)
set(ESC_DEFAULT_NAMES
  PID_SPEED_KP_DEFAULT PID_SPEED_KI_DEFAULT
  FRICTION_FF_STATIC_MA_DEFAULT FRICTION_FF_KINETIC_MA_DEFAULT
  SPEED_REV_FILTER_DEFAULT SPEED_EDGE_EST_DEFAULT
  DSC_ENABLE_DEFAULT DSC_FF_RUN_MA_DEFAULT DSC_FF_PER_KRPM_MA_DEFAULT
  DSC_START_FROM_MA_DEFAULT DSC_START_CAP_MA_DEFAULT DSC_START_SLOPE_MA_DEFAULT
  DSC_START_HOLD_MS_DEFAULT DSC_PCAP_MA_DEFAULT DSC_PCAP_MS_DEFAULT
  DSC_PCAP_UPDATES_DEFAULT DSC_PCAP_FALLBACK_MS_DEFAULT
  DSC_HOLD_DECAY_MA_DEFAULT DSC_WATCH_MS_DEFAULT DSC_PRELOAD_PCT_DEFAULT)

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
