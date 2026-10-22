set(ops_sources
  src/shared/src/motor_protocol.c src/tools/lib/ik_config.c
  src/tools/lib/drive_ops.c src/tools/lib/drive_keyboard.c src/tools/lib/drive_record.c
  src/tools/lib/pulse_run.c src/tools/lib/esc_ops.c src/tools/lib/tune_analysis.c
  src/tools/lib/esc_defaults.c src/tools/lib/ik_link.c src/tools/lib/ocd.c src/tools/lib/elfsym.c
  src/tools/lib/startup_metrics.c)
set(ops_absolute_sources)
foreach(source ${ops_sources})
  list(APPEND ops_absolute_sources ${REPO}/${source})
endforeach()
add_library(ops_test_tools STATIC ${ops_absolute_sources})
target_include_directories(ops_test_tools PUBLIC ${REPO}/src/tools/lib ${REPO}/src/shared/can ${REPO}/src/shared/inc ${CMAKE_BINARY_DIR}/gen)
target_link_libraries(ops_test_tools PUBLIC m)
host_test(test_drive_keys LIBS ops_test_tools)
host_test(test_drive_record LIBS ops_test_tools)
host_test(test_drive_stage LIBS ops_test_tools
  DEFS FAKE_OCD="$<TARGET_FILE:fake_openocd>" FAKE_ELF="$<TARGET_FILE:fake_ik_syms>"
  DEPENDS fake_openocd fake_ik_syms)
foreach(tool move_test keyboard_drive tune_ground)
  add_executable(ops_${tool} ${REPO}/src/tools/${tool}.c)
  target_link_libraries(ops_${tool} PRIVATE ops_test_tools)
endforeach()
host_test(test_drive_cli LIBS ops_test_tools util
  DEFS MOVE_TOOL="$<TARGET_FILE:ops_move_test>" KEYBOARD_TOOL="$<TARGET_FILE:ops_keyboard_drive>"
       TUNE_TOOL="$<TARGET_FILE:ops_tune_ground>" FAKE_OCD="$<TARGET_FILE:fake_openocd>"
       FAKE_ELF="$<TARGET_FILE:fake_ik_syms>"
  DEPENDS ops_move_test ops_keyboard_drive ops_tune_ground fake_openocd fake_ik_syms)
add_test(NAME test_drive_stage.rpc COMMAND test_drive_stage --rpc)
add_test(NAME test_drive_cli.rpc COMMAND test_drive_cli --rpc)
set_tests_properties(test_drive_stage.rpc test_drive_cli.rpc PROPERTIES SKIP_RETURN_CODE 77)
target_link_options(test_startup_metrics PRIVATE -Wl,--wrap=ik_link_rows)
host_test(test_friction_run
  SOURCES src/tools/lib/friction_run.c src/tools/lib/friction_fit.c
  LIBS ops_test_tools)
foreach(boundary pulse_now_s pulse_sleep_s ik_set_mode ik_drive ik_drive_current ik_write ik_read
                 ik_status ik_check_fault ik_link_rows ik_rows)
  target_link_options(test_friction_run PRIVATE -Wl,--wrap=${boundary})
endforeach()
add_executable(ops_friction_map ${REPO}/src/tools/friction_map.c
  ${REPO}/src/tools/lib/friction_run.c ${REPO}/src/tools/lib/friction_fit.c)
target_link_libraries(ops_friction_map PRIVATE ops_test_tools)
add_executable(ops_esc_tool ${REPO}/src/tools/esc_tool.c
  ${REPO}/src/tools/lib/esc_ctl.c ${REPO}/src/tools/lib/capture_analysis.c)
target_link_libraries(ops_esc_tool PRIVATE ops_test_tools)
host_test(test_friction_cli
  LIBS ops_test_tools
  DEFS FRICTION_TOOL="$<TARGET_FILE:ops_friction_map>" ESC_TOOL="$<TARGET_FILE:ops_esc_tool>"
  DEPENDS ops_friction_map ops_esc_tool)
host_test(test_pulse_config LIBS ops_test_tools)
foreach(boundary ik_config_exchange ik_set_mode ik_read ik_write ik_check_fault ik_rows)
  target_link_options(test_pulse_config PRIVATE -Wl,--wrap=${boundary})
endforeach()
host_test(test_friction_map
  SOURCES src/tools/lib/friction_fit.c LIBS ops_test_tools)
foreach(boundary ik_link_open ik_link_close ik_read drive_preflight friction_stop ik_set_mode
                 pulse_now_s pulse_sleep_s friction_speed_move friction_breakaway_run)
  target_link_options(test_friction_map PRIVATE -Wl,--wrap=${boundary})
endforeach()
