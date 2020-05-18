cmake_minimum_required(VERSION 3.10)
project(control_panel)

set(MICRO_UTILS_DIR ../../../micro-utils)

add_subdirectory(${MICRO_UTILS_DIR}/test/gtest ${MICRO_UTILS_DIR}/test/build/gtest)

include_directories(
    "${MICRO_UTILS_DIR}/include"
    "src"
    "../include"
)

file(GLOB SOURCES
    "${MICRO_UTILS_DIR}/src/*.c"
    "${MICRO_UTILS_DIR}/src/*.cpp"
    "../src/*.c"
    "../src/*.cpp"
    "src/*.cpp"
)

add_executable(${CMAKE_PROJECT_NAME}_test ${SOURCES})

add_test(NAME ${CMAKE_PROJECT_NAME}_test COMMAND ${CMAKE_PROJECT_NAME}_test)

target_link_libraries(${CMAKE_PROJECT_NAME}_test PUBLIC gtest)
