cmake_minimum_required(VERSION 3.10)

project(8080Emulator C)

set(CMAKE_C_STANDARD 99)

include_directories(${PROJECT_SOURCE_DIR}/source)

file(GLOB SOURCE_FILES ${PROJECT_SOURCE_DIR}/source/*.c)
add_executable(8080Emulator ${SOURCE_FILES})

set_target_properties(8080Emulator PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
)

set(CMAKE_BUILD_TYPE Debug)