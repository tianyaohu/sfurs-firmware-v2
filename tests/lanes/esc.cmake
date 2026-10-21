set(esc_app_cases
  test_alignment_detects_reversed_wiring
  test_alignment_detects_stuck_rotor
  test_streaming_and_watchdog
  test_stop_and_status_reply
  test_fault_latches_until_stop
  test_i2t_derates_and_recovers
  test_config_only_when_idle
  test_stop_aborts_alignment
  test_stop_clears_finished_fault
  test_ident_reports_validity_record
  test_capture_steps_id_and_reads_out
  test_capture_stop_and_busy
  test_friction_feedforward
  test_parks_at_zero_command
  test_startup_loop_config_and_failed_latch
  test_startup_speed_plausibility
  test_speed_loop_recorder
  test_recorder_idle_arm_and_early_stop
  test_recorder_dsc_stop_drops_loop_terms)

foreach(wheel NE SE SW NW)
  string(TOLOWER "${wheel}" suffix)
  if(wheel STREQUAL "NE")
    set(app_test test_esc_drive_app)
  else()
    set(app_test test_esc_drive_app_${suffix})
  endif()
  host_test(${app_test}
    SOURCES src/esc/motor_config/Src/drive_app.c src/esc/motor_config/Src/motor_cmd.c
            src/shared/src/motor_protocol.c
    INCLUDES tests/esc_drive_app_fakes src/esc/motor_config/Inc src/esc/Inc src/shared/can src/shared/inc
    DEFS MOTOR_${wheel} DRIVE_APP_VALIDITY=fake_validity
    LIBS m)
  target_compile_options(${app_test} PRIVATE
    -include "${REPO}/tests/esc_drive_app_fakes/test_hooks.h")
  # Separate processes preserve the firmware's power-on static state per scenario.
  foreach(case IN LISTS esc_app_cases)
    add_test(NAME ${app_test}.${case} COMMAND ${app_test} ${case})
  endforeach()
endforeach()

set(esc_sdk src/esc/motor_config/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any)
host_test(test_esc_speed_ctrl
  SOURCES src/esc/motor_config/Src/drive_speed_ctrl.c ${esc_sdk}/Src/pid_regulator.c
  INCLUDES tests/esc_speed_ctrl_fakes src/esc/motor_config/Inc ${esc_sdk}/Inc)
host_test(test_esc_start_reference
  DEFS ESC_SOURCE_DIR="${REPO}/src/esc/motor_config/Src")
host_test(test_esc_blink_can
  SOURCES tests/esc_blink/can_boot.c
  INCLUDES tests/esc_blink src/shared/inc
  DEFS ESC_BLINK_CAN_TEST)

add_executable(esc_blink_image esc_blink/image_embed.c ${REPO}/src/tools/lib/crc32.c)
target_include_directories(esc_blink_image PRIVATE ${REPO}/tests ${REPO}/src/tools/lib)
find_program(ESC_ARM_CC arm-none-eabi-gcc HINTS "$ENV{HOME}/.local/bin")
find_program(ESC_ARM_OBJCOPY arm-none-eabi-objcopy HINTS "$ENV{HOME}/.local/bin")
set(esc_blink_tools)
if(ESC_ARM_CC)
  list(APPEND esc_blink_tools --cc "${ESC_ARM_CC}")
endif()
if(ESC_ARM_OBJCOPY)
  list(APPEND esc_blink_tools --objcopy "${ESC_ARM_OBJCOPY}")
endif()
add_test(NAME test_esc_blink_build
  COMMAND sh ${REPO}/tests/esc_blink/build.sh ${esc_blink_tools}
    --image-embed $<TARGET_FILE:esc_blink_image> --output ${CMAKE_CURRENT_BINARY_DIR}/esc_blink)
set_tests_properties(test_esc_blink_build PROPERTIES SKIP_RETURN_CODE 77)

foreach(target test_esc_drive_app test_esc_drive_app_se test_esc_drive_app_sw
    test_esc_drive_app_nw test_esc_speed_ctrl test_esc_start_reference
    test_esc_blink_can esc_blink_image)
  set_target_properties(${target} PROPERTIES C_EXTENSIONS OFF)
endforeach()

# Protocol v1 command lifecycle (pure; no MCSDK fakes needed).
host_test(test_motor_cmd
  SOURCES src/esc/motor_config/Src/motor_cmd.c src/shared/src/motor_protocol.c
  INCLUDES src/esc/motor_config/Inc src/shared/inc)
