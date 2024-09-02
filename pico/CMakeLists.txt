cmake_minimum_required(VERSION 3.20)

set(PROJECT gcisensors)
set(PICO_BOARD pico) # pico or pico_w
set(PICO_SDK_PATH "$ENV{HOME}/github/pico-sdk")
# set(ENV{PATH} "$ENV{HOME}/github/gcc-arm/bin:$ENV{PATH}")
set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Path to the PICO SDK")
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

include(FetchContent)

# Need ASM for tinyUSB and picotool to work
project(${PROJECT}
    VERSION "2023.10.28"
    LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

# string (APPEND CMAKE_CXX_FLAGS " -Wdeprecated-enum-enum-conversion ")
string (APPEND CMAKE_CXX_FLAGS_RELEASE " -O3 ")
string (APPEND CMAKE_C_FLAGS_RELEASE " -O3 ")
string(PREPEND CMAKE_EXE_LINKER_FLAGS " -Wdeprecated-enum-enum-conversion -Wl,--print-memory-usage ")
add_compile_options( -Wall -Wconversion -Wsign-conversion -Wall )
# cmake_cxx_flags( -Wdeprecated-enum-enum-conversion )
# add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wdeprecated-enum-enum-conversion>)
# add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wdeprecated-enum-enum-conversion>)

# Initialize the SDK
pico_sdk_init()

message(STATUS ">> test: ${PICO_SDK}  ${PICO_BOARD} ${ARDUINO}")

if(PICO_SDK)
    add_definitions(-DPICO_SDK=1)
endif()

if(PROJECT_IS_TOP_LEVEL)
    cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
    cmake_host_system_information(RESULT OSN QUERY OS_NAME)
    cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
    cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

    message(STATUS "-------------------------------------")
    message(STATUS "  Project: ${PROJECT_NAME}")
    message(STATUS "  Hardware: ${PICO_BOARD}")
    message(STATUS "  C++ ${CMAKE_CXX_STANDARD}")
    message(STATUS "  C ${CMAKE_C_STANDARD}")
    message(STATUS "-------------------------------------")
    message(STATUS " ${HOST}")
    message(STATUS " ${OSN}: ${OS_VERSION}")
    message(STATUS " ${PROC}")
    message(STATUS "-------------------------------------")

    set(BUILD_EXAMPLES ON)
else()
    message(STATUS "-> ${PROJECT_NAME} is submodule")
    set(BUILD_EXAMPLES OFF)
endif()

LIST(APPEND pico_libs
    pico_stdlib
    hardware_i2c)


add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ../src .)
target_link_libraries(${PROJECT_NAME} INTERFACE ${pico_libs})

if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
