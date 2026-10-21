# Host tests: real firmware and tool C compiled against fakes at documented
# boundaries (HAL calls, CAN frames, OpenOCD's Tcl RPC). No hardware.
#
#   cmake -S tests -B build/host && cmake --build build/host && ctest --test-dir build/host
#
# Add a test with host_test(); sources and include dirs are relative to the repo root.
cmake_minimum_required(VERSION 3.16)
project(sfurs_host_tests C)
enable_testing()

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(CMAKE_C_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
add_compile_options(-O2 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer)
add_link_options(-fsanitize=address,undefined -no-pie)

# gen_esc_defaults.h: drive_parameters.h defaults for the PC tools.
include(${REPO}/src/tools/esc_defaults.cmake)
generate_esc_defaults(${CMAKE_BINARY_DIR}/gen/gen_esc_defaults.h)

# host_test(name SOURCES a.c b.c [INCLUDES dir...] [DEFS X=1...] [LIBS m...] [DEPENDS target...])
function(host_test name)
  cmake_parse_arguments(T "" "" "SOURCES;INCLUDES;DEFS;LIBS;DEPENDS" ${ARGN})
  set(srcs ${CMAKE_CURRENT_SOURCE_DIR}/${name}.c)
  foreach(s ${T_SOURCES})
    list(APPEND srcs ${REPO}/${s})
  endforeach()
  add_executable(${name} ${srcs})
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gen)
  foreach(i ${T_INCLUDES})
    target_include_directories(${name} PRIVATE ${REPO}/${i})
  endforeach()
  target_compile_definitions(${name} PRIVATE ${T_DEFS})
  target_link_libraries(${name} PRIVATE ${T_LIBS})
  if(T_DEPENDS)
    add_dependencies(${name} ${T_DEPENDS})
  endif()
  add_test(NAME ${name} COMMAND ${name})
endfunction()

add_executable(fake_openocd fake_openocd.c)

# One lanes/<area>.cmake per area calls host_test(); keeps parallel work conflict-free.
file(GLOB lane_files ${CMAKE_CURRENT_SOURCE_DIR}/lanes/*.cmake)
foreach(f ${lane_files})
  include(${f})
endforeach()
