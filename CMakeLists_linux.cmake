# v3.14 required for FetchContent_MakeAvailable
cmake_minimum_required(VERSION 3.21)

PROJECT(gcisensors
    VERSION "2023.07.01"
    LANGUAGES CXX
)

include(FetchContent)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(PROJECT_IS_TOP_LEVEL)
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

    # set(BUILD_EXAMPLES ON)
    set(BUILD_GTESTS OFF)

    # GTest -----------------
    # FetchContent_Declare(
    #     gtest
    #     URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    # )
    # LIST(APPEND libs gtest)
else()
    message(STATUS "-> ${PROJECT_NAME} is submodule")
    # set(BUILD_EXAMPLES OFF)
    set(BUILD_GTESTS OFF)
endif()

# FetchContent_Declare(
#     gciwire
#     GIT_REPOSITORY "https://github.com/the-guild-of-calamitous-intent/gciWire.git"
#     GIT_TAG "origin/main"
# )

# LIST(APPEND libs gciwire)

# FetchContent_MakeAvailable( ${libs} )

# message(STATUS "[ Wire ]=======================")
# message(STATUS "> POPULATED: ${gciwire_POPULATED}")
# message(STATUS "> SRC_DIR: ${gciwire_SOURCE_DIR}")
# message(STATUS "> BIN_DIR: ${gciwire_BINARY_DIR}")
# message(STATUS "===============================")

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE src)
# target_link_libraries(${PROJECT_NAME} INTERFACE gciwire)

# Examples ====================================================================
# message(STATUS "Building gciSensors examples is ${BUILD_EXAMPLES}")
# if (BUILD_EXAMPLES)
#     add_subdirectory(examples)
# endif()

# add_subdirectory(examples/pico)

# message(STATUS "Building gciSensors gtests is ${BUILD_GTESTS}")
# if (BUILD_GTESTS)
#     add_subdirectory(gtests)
# endif()
