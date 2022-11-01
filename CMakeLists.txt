# v3.14 required for FetchContent_MakeAvailable
cmake_minimum_required(VERSION 3.14)

PROJECT(cgiSensors
    VERSION "0.1.0"
    LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
cmake_host_system_information(RESULT OSN QUERY OS_NAME)
cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

message(STATUS "-------------------------------------")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  Version: ${CMAKE_PROJECT_VERSION}")
message(STATUS "  C++ ${CMAKE_CXX_STANDARD}")
message(STATUS "-------------------------------------")
message(STATUS " ${HOST}")
message(STATUS " ${OSN}: ${OS_VERSION}")
message(STATUS " ${PROC}")
message(STATUS "-------------------------------------")

# set(CMAKE_CXX_COMPILER -Wunused ) doesn't work

# Add the library target
add_library(${PROJECT_NAME}
    src/bmp390/bmp3.cpp
    src/bmp390/bosch/bmp3imp.cpp
    src/lis3mdl/lis3mdl.cpp
    src/lsm6dsox/lsm6dsox.cpp
    src/sensor.cpp
)
target_include_directories(
    ${PROJECT_NAME} PUBLIC
    #$<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/mock>
    $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/linux>
    $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        wirelib
)

if (UNIX)
    if (APPLE)
        message(STATUS "Detected APPLE, using mock i2c")
        add_subdirectory(mock)
        target_include_directories(
            ${PROJECT_NAME} PUBLIC
            $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/mock>
            $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/src>
        )
    else ()
        message(STATUS "Detected LINUX, using linux i2c")
        add_subdirectory(linux)
        target_include_directories(
            ${PROJECT_NAME} PUBLIC
            $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/linux>
            $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/src>
        )
    endif (APPLE)
endif (UNIX)

# add_subdirectory(examples)

# Examples ====================================================================
option(EXAMPLES "Examples" ON)
message(STATUS "Building examples is ${EXAMPLES}")
if (EXAMPLES)
    add_subdirectory(examples)
endif()

