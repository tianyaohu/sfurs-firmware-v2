host_test(test_ik_drive
  SOURCES src/nucleo/CM7/Core/Src/ik_drive.c src/shared/src/rover_kinematics.c
  INCLUDES src/nucleo/CM7/Core/Inc src/shared/inc src/shared/can
  LIBS m)

# Each drive case needs the firmware's initial static state.
foreach(case
    streams_and_slews_to_target
    release_brakes_then_stops
    lost_heartbeat_brakes_then_stops
    reverses_without_stop
    zero_twist_holds
    bad_input_and_saturation
    raw_frames_only_when_stopped
    tx_failures_counted
    custom_acceleration)
  add_test(NAME test_ik_drive.${case} COMMAND test_ik_drive ${case})
endforeach()

host_test(test_ik_telemetry
  SOURCES src/nucleo/CM7/Core/Src/ik_telemetry.c
  INCLUDES src/nucleo/CM7/Core/Inc src/shared/can)

host_test(test_rover_kinematics
  SOURCES src/shared/src/rover_kinematics.c
  INCLUDES src/shared/inc
  LIBS m)
