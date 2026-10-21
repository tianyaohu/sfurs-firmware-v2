add_executable(startup_report ${CMAKE_CURRENT_SOURCE_DIR}/startup_report.c)
target_link_libraries(startup_report PRIVATE tools)
