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
    fault_stop_starts_a_new_session)
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
