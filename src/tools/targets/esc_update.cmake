add_executable(esc_update ${CMAKE_CURRENT_SOURCE_DIR}/esc_update.c)
target_link_libraries(esc_update PRIVATE tools)
get_filename_component(ESC_UPDATE_BUILD "${CMAKE_CURRENT_SOURCE_DIR}/../../build" ABSOLUTE)
target_compile_definitions(tools PRIVATE FLASHIMG_DEFAULT_BUILD="${ESC_UPDATE_BUILD}")
