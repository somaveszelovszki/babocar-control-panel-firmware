cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB SOURCES
    "src/*.cpp"
)

add_executable(${PROJECT_NAME}_test ${SOURCES})

add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)

target_link_libraries(${PROJECT_NAME}_test PUBLIC gtest ${PROJECT_NAME})
