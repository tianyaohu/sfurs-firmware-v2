host_test(test_ik_drive
  SOURCES src/nucleo/CM7/Core/Src/ik_drive.c src/shared/src/rover_kinematics.c src/shared/src/friction_ff.c
          src/shared/src/motor_protocol.c src/esc/motor_config/Src/motor_cmd.c
  INCLUDES src/nucleo/CM7/Core/Inc src/shared/inc src/shared/can src/esc/motor_config/Inc
  LIBS m)

# Each drive case needs the firmware's initial static state.
foreach(case
    needs_enable_and_heartbeat
    links_then_starts_together
    streams_and_slews_to_target
    release_brakes_then_stops
    lost_heartbeat_brakes_then_stops
    reverses_without_stop
    zero_twist_holds
    bad_input_and_saturation
    raw_frames_only_when_stopped
    tx_failures_counted
    custom_acceleration
    lost_wheel_stops_the_robot
    faulted_wheel_stops_until_cleared
    resyncs_a_moved_baseline
    feed_forward_rides_in_speed_frames
    lost_start_reply_stays_invalid
    reply_latency_histogram
    config_reply_stored
    current_mode_frames
    rejected_twist_has_no_ff
    breakaway_boost_until_two_wheels_turn
    breakaway_boost_time_limit
    fault_stop_starts_a_new_session
    new_session_learns_latched_cause
    esc_restart_is_counted)
  add_test(NAME test_ik_drive.${case} COMMAND test_ik_drive ${case})
endforeach()
host_test(test_ik_telemetry
  SOURCES src/nucleo/CM7/Core/Src/ik_telemetry.c
  INCLUDES src/nucleo/CM7/Core/Inc src/shared/can)

host_test(test_rover_kinematics
  SOURCES src/shared/src/rover_kinematics.c
  INCLUDES src/shared/inc
  LIBS m)

host_test(test_friction_ff
  SOURCES src/shared/src/friction_ff.c src/shared/src/rover_kinematics.c
  INCLUDES src/shared/inc
  LIBS m)

# skynet serial link into the real drive mailbox; each case needs fresh static state.
host_test(test_skynet_link
  SOURCES src/nucleo/CM7/Core/Src/skynet_link.c src/nucleo/CM7/Core/Src/ik_drive.c
          src/shared/src/rover_kinematics.c src/shared/src/friction_ff.c src/shared/src/motor_protocol.c
  INCLUDES src/nucleo/CM7/Core/Inc src/shared/inc src/shared/can
  LIBS m
  CASES crc_matches_ccitt_false batch_drives_this_robot other_robots_leave_mailbox_alone
        finds_batches_after_garbage_and_splits corrupt_record_discards_its_batch ids_out_of_order_discard_batch
        truncated_tail_is_not_a_batch reserved_bits_reject_record full_scale_is_clamped_to_drive_limits
        silence_stops_and_restart_drives stale_bytes_are_dropped decoded_command_does_not_outlive_a_stall
        record_cut_by_silence_is_not_completed ring_overflow_is_counted overrun_discards_open_batch
        refuses_current_mode_and_busy_drive another_host_writing_takes_over
        other_writer_seen_before_next_batch tick_wrap_and_early_stamps fault_latches_until_acknowledged
        acknowledge_before_fault_does_not_clear_it button_bounce_is_one_acknowledge
        resume_during_stop_keeps_driving damaged_streams_apply_only_sent_commands)

# The link against the whole drive: real ik_drive.c and four real ESC lifecycles (test_ik_drive.c fixture).
host_test(test_skynet_link_drive
  SOURCES src/nucleo/CM7/Core/Src/skynet_link.c src/nucleo/CM7/Core/Src/ik_drive.c
          src/shared/src/rover_kinematics.c src/shared/src/friction_ff.c src/shared/src/motor_protocol.c
          src/esc/motor_config/Src/motor_cmd.c
  INCLUDES src/nucleo/CM7/Core/Inc src/shared/inc src/shared/can src/esc/motor_config/Inc
  LIBS m
  CASES dropped_byte_applies_no_command fault_waits_for_operator_0 fault_waits_for_operator_1
        fault_waits_for_operator_2 fault_waits_for_operator_3 inherited_current_session_is_refused
        queued_record_goes_stale cut_record_does_not_absorb_halt)
