cmake_minimum_required(VERSION 3.14)
project(rs_i2t_tests LANGUAGES C CXX)

# GoogleTest requires at least C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
# Extracted files get the extraction time (CMake >= 3.24 default).
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_library(rs_i2t STATIC
  ../rs_i2t.c
)
target_include_directories(rs_i2t PUBLIC
  ..
)

add_executable(test_rs_i2t
  test_rs_i2t.cc
)
target_link_libraries(test_rs_i2t PRIVATE
  rs_i2t
  GTest::gtest_main
)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_rs_i2t)
