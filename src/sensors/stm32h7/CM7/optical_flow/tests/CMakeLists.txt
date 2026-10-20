cmake_minimum_required(VERSION 3.14)
project(optical_flow_tests LANGUAGES C CXX)

# The test harness is C++.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
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
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
)
# Required on Windows/MSVC: make gtest use the same dynamic CRT we do.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(test_optical_flow
  test_optical_flow.cc
)
target_include_directories(test_optical_flow PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(test_optical_flow PRIVATE
  GTest::gtest_main
)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_optical_flow)
