cmake_minimum_required(VERSION 3.16)
project(can_protocol_test C)

add_executable(can_protocol_test
    test_can_protocol.c
    ../main/can_protocol.c
)
target_include_directories(can_protocol_test PRIVATE ../main)
target_compile_options(can_protocol_test PRIVATE -Wall -Wextra -Werror)
enable_testing()
add_test(NAME can_protocol_test COMMAND can_protocol_test)
