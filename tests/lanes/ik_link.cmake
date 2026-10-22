add_executable(fake_ik_syms fake_ik_syms.c)
target_compile_options(fake_ik_syms PRIVATE -fno-sanitize=all)
target_link_options(fake_ik_syms PRIVATE -fno-sanitize=all)

host_test(test_ik_link
  SOURCES src/tools/lib/ik_link.c src/tools/lib/ocd.c src/tools/lib/elfsym.c src/shared/src/motor_protocol.c
  INCLUDES src/tools/lib src/shared/can src/shared/inc
  DEFS FAKE_OCD="$<TARGET_FILE:fake_openocd>" FAKE_ELF="$<TARGET_FILE:fake_ik_syms>"
  DEPENDS fake_openocd fake_ik_syms)
host_test(test_esc_ops
  SOURCES src/tools/lib/esc_ops.c INCLUDES src/tools/lib LIBS m)
host_test(test_esc_ctl
  SOURCES src/tools/lib/esc_ctl.c src/tools/lib/capture_analysis.c src/shared/src/motor_protocol.c
  INCLUDES src/tools/lib src/shared/can src/shared/inc LIBS ops_test_tools m)
host_test(test_esc_defaults
  SOURCES src/tools/lib/esc_defaults.c src/tools/lib/ik_link.c src/tools/lib/ocd.c src/tools/lib/elfsym.c
  INCLUDES src/tools/lib src/shared/can src/shared/inc)
host_test(test_sweep
  SOURCES src/tools/lib/ik_config.c src/shared/src/motor_protocol.c src/tools/sweep.c src/tools/lib/ik_link.c src/tools/lib/ocd.c src/tools/lib/elfsym.c
          src/tools/lib/pulse_run.c src/tools/lib/esc_ops.c src/tools/lib/esc_defaults.c
  INCLUDES src/tools/lib src/shared/can src/shared/inc DEFS main=sweep_main LIBS m)
host_test(test_config_exchange
  SOURCES src/tools/lib/ik_config.c src/tools/lib/ik_link.c src/tools/lib/ocd.c src/tools/lib/elfsym.c src/shared/src/motor_protocol.c
  INCLUDES src/tools/lib src/shared/can src/shared/inc
  DEFS FAKE_OCD="$<TARGET_FILE:fake_openocd>" FAKE_ELF="$<TARGET_FILE:fake_ik_syms>"
  DEPENDS fake_openocd fake_ik_syms)
set_tests_properties(test_config_exchange PROPERTIES SKIP_RETURN_CODE 77)
add_test(NAME test_ik_link.rpc COMMAND test_ik_link --rpc)
set_tests_properties(test_ik_link.rpc PROPERTIES SKIP_RETURN_CODE 77)
foreach(boundary ik_link_open ik_link_close ik_read ik_write ik_set_mode ik_check_fault ik_rows ik_drive)
  target_link_options(test_sweep PRIVATE -Wl,--wrap=${boundary})
endforeach()
