# PC tools (talk to the Nucleo and ESCs through OpenOCD). No hardware needed to build.
#
#   cmake -S src/tools -B build/tools -G Ninja && cmake --build build/tools
#
# Binaries land in build/tools/.
cmake_minimum_required(VERSION 3.16)
project(sfurs_tools C)

set(CMAKE_C_STANDARD 11)
add_compile_options(-O2 -g -Wall -Wextra -Werror)

include(${CMAKE_CURRENT_SOURCE_DIR}/esc_defaults.cmake)
generate_esc_defaults(${CMAKE_BINARY_DIR}/gen/gen_esc_defaults.h)

set(LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib)
file(GLOB LIB_SOURCES CONFIGURE_DEPENDS ${LIB}/*.c)  # every lib/*.c is part of the library
add_library(tools STATIC ${LIB_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/../shared/src/motor_protocol.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../shared/src/friction_ff.c ${CMAKE_CURRENT_SOURCE_DIR}/../shared/src/rover_kinematics.c)
target_include_directories(tools PUBLIC ${LIB} ${CMAKE_CURRENT_SOURCE_DIR}/../shared/can ${CMAKE_CURRENT_SOURCE_DIR}/../shared/inc ${CMAKE_BINARY_DIR}/gen)
target_link_libraries(tools PUBLIC m)

# One targets/<tool>.cmake per tool (add_executable + target_link_libraries(... tools)),
# so parallel work on different tools does not conflict.
file(GLOB tool_targets ${CMAKE_CURRENT_SOURCE_DIR}/targets/*.cmake)
foreach(f ${tool_targets})
  include(${f})
endforeach()
