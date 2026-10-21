host_test(test_capture_analysis
  SOURCES src/tools/lib/capture_analysis.c INCLUDES src/tools/lib LIBS m)
host_test(test_tune_analysis
  SOURCES src/tools/lib/tune_analysis.c INCLUDES src/tools/lib LIBS m)
# startup_metrics.c comes with ops_test_tools (lanes/ops.cmake), which also builds move_test.
# Link the real CLI helpers; wrap only acquisition/motion boundaries in this test.
host_test(test_startup_metrics INCLUDES src/tools/lib LIBS ops_test_tools)
foreach(boundary ik_link_open ik_link_close drive_preflight ik_write ik_record_start
                 drive_stage_run drive_records_read ik_record_read)
  target_link_options(test_startup_metrics PRIVATE -Wl,--wrap=${boundary})
endforeach()
