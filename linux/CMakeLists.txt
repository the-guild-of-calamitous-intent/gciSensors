cmake_minimum_required(VERSION 3.20)

project(gcisensors
    VERSION "2023.10.28"
    LANGUAGES C CXX)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

include(FetchContent)


string (APPEND CMAKE_CXX_FLAGS_RELEASE " -O3 ")
string (APPEND CMAKE_C_FLAGS_RELEASE " -O3 ")
# string(PREPEND CMAKE_EXE_LINKER_FLAGS " -Wl,--print-memory-usage ")

if(PROJECT_IS_TOP_LEVEL)
    cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
    cmake_host_system_information(RESULT OSN QUERY OS_NAME)
    cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
    cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

    message(STATUS "-------------------------------------")
    message(STATUS "  Project: ${PROJECT_NAME}")
    message(STATUS "  C++ ${CMAKE_CXX_STANDARD}")
    message(STATUS "  C ${CMAKE_C_STANDARD}")
    message(STATUS "-------------------------------------")
    message(STATUS " Host: ${HOST}")
    message(STATUS " ${OSN}: ${OS_VERSION}")
    message(STATUS " ${PROC}")
    message(STATUS "-------------------------------------")

    set(BUILD_EXAMPLES ON)
else()
    message(STATUS "-> ${PROJECT_NAME} is submodule")
    set(BUILD_EXAMPLES OFF)
endif()

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE src .)
target_link_libraries(${PROJECT_NAME} INTERFACE i2c)

if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
