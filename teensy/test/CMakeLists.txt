# Arduino'dan bagimsiz firmware modulleri icin host testleri.
#
#   cmake -S teensy/test -B build/test
#   cmake --build build/test
#   ctest --test-dir build/test --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(stingray_firmware_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../stingray_teensy)

add_executable(test_frame_codec
    test_frame_codec.cpp
    ${FW_DIR}/frame_codec.cpp
    ${FW_DIR}/dsp_filter.cpp)
target_include_directories(test_frame_codec PRIVATE ${FW_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(test_frame_codec PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Werror
    -fsanitize=address,undefined -fno-omit-frame-pointer)
target_link_options(test_frame_codec PRIVATE -fsanitize=address,undefined)

enable_testing()
add_test(NAME frame_codec COMMAND test_frame_codec)
