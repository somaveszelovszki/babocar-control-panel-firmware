cmake_minimum_required(VERSION 3.25)
project(control_panel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB SOURCES
    "src/*.c"
    "src/*.cpp"
)

add_library(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PUBLIC etl::etl micro_utils)

if (BUILD_TESTS)
    message("Tests are enabled")
    add_compile_definitions(COMPILE_ALL_TRACKS)
    enable_testing()
    add_subdirectory(test)
endif()
