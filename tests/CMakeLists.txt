cmake_minimum_required(VERSION 3.16)
project(sfurs_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(test_rover_kinematics
    test_rover_kinematics.c
    ../src/shared/src/rover_kinematics.c)
target_include_directories(test_rover_kinematics PRIVATE ../src/shared/inc)
target_compile_options(test_rover_kinematics PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(test_rover_kinematics PRIVATE m)
add_test(NAME rover_kinematics COMMAND test_rover_kinematics)
