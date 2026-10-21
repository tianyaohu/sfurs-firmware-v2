set(fwu_shared_sources
  src/shared/src/can_fwu_protocol.c
  src/shared/src/can_fwu_boot.c)
set(fwu_host_source src/nucleo/CM7/Core/Src/can_fwu_host.c)
set(fwu_includes src/shared/inc src/nucleo/CM7/Core/Inc)

host_test(test_can_fwu
  SOURCES ${fwu_shared_sources} ${fwu_host_source}
  INCLUDES ${fwu_includes})
host_test(test_fwu_updater
  SOURCES ${fwu_shared_sources} ${fwu_host_source}
    src/nucleo/CM7/Core/Src/fwu_updater.c
  INCLUDES ${fwu_includes})

# The default host_test entry runs the first case; each other case gets the
# same fresh process that the original Python runner provided.
foreach(case
  test_full_10k_session
  test_corrupted_transfer_faults
  test_motor_size_idle_rejected
  test_sequence_and_idle_timeouts
  test_padded_and_tiny_sessions
  test_host_ack_replay_correlation
  test_target_retries_idempotent
  test_invalid_devices_and_send_timeout
  test_lost_enter_ack_recovery
  test_minimum_size_and_accepted_traffic
  test_tick_and_session_wrap
)
  add_test(NAME test_can_fwu.${case} COMMAND test_can_fwu ${case})
endforeach()

foreach(case
  test_enter_retries_until_answered
  test_failure_latches_until_new_request
  test_boot_entry_request
  test_fresh_clock_after_begin
  test_button_hold_arms
  test_button_released_early_never_arms
  test_completed_transfer_does_not_restart
)
  add_test(NAME test_fwu_updater.${case} COMMAND test_fwu_updater ${case})
endforeach()

host_test(test_fwu_validity INCLUDES src/shared/inc)
host_test(test_fwu_verified_recovery
  SOURCES ${fwu_shared_sources} INCLUDES src/shared/inc)
host_test(test_fwu_motor_image
  SOURCES src/shared/src/can_fwu_protocol.c INCLUDES src/shared/inc)
host_test(test_esc_boot_helpers
  SOURCES ${fwu_shared_sources} INCLUDES src/shared/inc)
set_tests_properties(test_esc_boot_helpers PROPERTIES
  ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")
host_test(test_boot_helpers
  SOURCES ${fwu_shared_sources}
  INCLUDES src/esc/bootloader src/shared/inc)
