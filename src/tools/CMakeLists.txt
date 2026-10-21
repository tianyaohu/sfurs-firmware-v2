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
add_library(tools STATIC
  ${LIB}/ocd.c ${LIB}/elfsym.c ${LIB}/crc32.c ${LIB}/ik_link.c ${LIB}/esc_ops.c ${LIB}/esc_defaults.c
  ${LIB}/capture_analysis.c ${LIB}/tune_analysis.c)
target_include_directories(tools PUBLIC ${LIB} ${CMAKE_CURRENT_SOURCE_DIR}/../shared/can ${CMAKE_BINARY_DIR}/gen)
target_link_libraries(tools PUBLIC m)

add_executable(esc_tool esc_tool.c)
target_link_libraries(esc_tool PRIVATE tools)
