cmake_minimum_required(VERSION 3.14)
project(imu_tests LANGUAGES C CXX)

# sensor_board.c is C; the test harness and gmock are C++.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
# Required on Windows/MSVC: make gtest use the same dynamic CRT we do.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# Unlike rs_i2t, we need gmock here for EXPECT_CALL on the HAL.
set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Module under test, built for the host against a stand-in HAL.
add_library(sensor_board STATIC
  ../sensor_board.c
)
target_include_directories(sensor_board PUBLIC
  # Listed first so #include "stm32h7xx_hal.h" resolves to our fake, not ST's.
  ${CMAKE_CURRENT_SOURCE_DIR}/fake_hal
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_executable(test_sensor_board
  test_sensor_board.cc
  hal_mock.cc
)
target_link_libraries(test_sensor_board PRIVATE
  sensor_board
  GTest::gmock_main
)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_sensor_board)
