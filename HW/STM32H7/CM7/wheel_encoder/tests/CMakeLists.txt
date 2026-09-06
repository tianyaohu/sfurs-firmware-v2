cmake_minimum_required(VERSION 3.14)
project(wheel_encoder_tests LANGUAGES C CXX)

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

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(test_wheel_encoder
	test_wheel_encoder.cc
	hal_mock.cc
	../wheel_encoder.c
)

target_include_directories(test_wheel_encoder PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/host_hal
	${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(test_wheel_encoder PRIVATE
	GTest::gtest_main
	GTest::gmock
)

enable_testing()

include(GoogleTest)
gtest_discover_tests(test_wheel_encoder)
