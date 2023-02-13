# v3.14 required for FetchContent_MakeAvailable
cmake_minimum_required(VERSION 3.14)

PROJECT(cgisensors
    VERSION "2023.02.10"
    LANGUAGES CXX
)

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

    set(BUILD_EXAMPLES ON)
    set(BUILD_GTESTS ON)
else()
    message(STATUS "-> ${PROJECT_NAME} is submodule")
    set(BUILD_EXAMPLES OFF)
    set(BUILD_GTESTS OFF)
endif()

# # CPM =========================================================================
# set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM.cmake")
# set(CPM_VERSION v0.35.1)
# if(NOT (EXISTS ${CPM_DOWNLOAD_LOCATION}))
#     message(STATUS "Downloading CPM.cmake")
#     file(DOWNLOAD
#         https://github.com/cpm-cmake/CPM.cmake/releases/download/${CPM_VERSION}/CPM.cmake
#         ${CPM_DOWNLOAD_LOCATION}
#     )
# else()
#     message(STATUS "CPM ${CPM_VERSION} ready")
# endif()
# include(${CPM_DOWNLOAD_LOCATION})

# CPMAddPackage(
#     NAME squaternion
#     VERSION 2022.08.20
#     GITHUB_REPOSITORY MomsFriendlyRobotCompany/squaternion.cpp
#     OPTIONS
#         "EXAMPLES OFF"
# )

include(FetchContent)

# if (linux)
#     # serialcomm -----------------
#     FetchContent_Declare(serialcomm
#     GIT_REPOSITORY "https://github.com/gecko-robotics/serialcomm.git"
#     GIT_TAG "origin/main"
#     SOURCE_DIR "${CMAKE_BINARY_DIR}/_deps/serialcomm"
#     )
#     set(EXAMPLES OFF CACHE INTERNAL "Dont build examples")
#     FetchContent_MakeAvailable(serialcomm)
#     if(serialcomm_POPULATED)
#         message(STATUS "=> Found serialcomm")
#     else()
#         message(STATUS "*** Didn't find serialcomm")
#     endif()
# endif(linux)

# gciMock -----------------
FetchContent_Declare(gciMock
  GIT_REPOSITORY "git@github.com:the-guild-of-calamitous-intent/gciMock.git"
  GIT_TAG "origin/main"
  SOURCE_DIR "${CMAKE_BINARY_DIR}/_deps/gcimock"
)
set(EXAMPLES OFF CACHE INTERNAL "Dont build examples")
FetchContent_MakeAvailable(gciMock)
if(gcimock_POPULATED)
    message(STATUS "=> Found gciMock")
else()
    message(STATUS "*** Didn't find gciMock")
endif()

# set(CMAKE_CXX_COMPILER -Wunused ) doesn't work

# Add the library target
add_library(${PROJECT_NAME}
    src/bmp390/bmp3.cpp
    src/lis3mdl/lis3mdl.cpp
    src/lsm6dsox/lsm6dsox.cpp
    src/sensor.cpp
)
# target_include_directories(
#     ${PROJECT_NAME} PUBLIC
#     #$<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/mock>
#     $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/linux>
#     $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/src>
# )

# target_link_libraries(${PROJECT_NAME}
#     PRIVATE
#         wirelib
#         # ${squaternion_SOURCE_DIR}
#         # $<$<PLATFORM_ID:Linux>:wirelib>
# )
# add_dependencies(${PROJECT_NAME} squaternion)

# target_include_directories(
#     ${PROJECT_NAME} PUBLIC
#     $<$<PLATFORM_ID:Linux>:BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/linux>
#     $<$<PLATFORM_ID:Apple>:$<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/mock>>
#     $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/src>
# )

# if (UNIX)
#     if (APPLE)
#         message(STATUS "Detected APPLE, using mock i2c")
#         message(STATUS ">> ${gciMock_SOURCE_DIR}/src")
#         message(STATUS ">> ${gcimock_SOURCE_DIR}/src")
#         # add_subdirectory(mock)
#         target_include_directories(
#             ${PROJECT_NAME} PUBLIC
#             # $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/mock>
#             $<BUILD_INTERFACE: ${gcimock_SOURCE_DIR}/src>
#             ${squaternion_SOURCE_DIR}/src
#         )
#     else ()
#         message(STATUS "Detected LINUX, using linux i2c")
#         # add_subdirectory(linux)
#         target_include_directories(
#             ${PROJECT_NAME} PUBLIC
#             # $<BUILD_INTERFACE: ${CMAKE_CURRENT_SOURCE_DIR}/linux>
#             $<BUILD_INTERFACE: ${serialcomm_SOURCE_DIR}/src>
#             ${squaternion_SOURCE_DIR}/src
#         )
#     endif (APPLE)
# endif (UNIX)

if (UNIX OR APPLE OR linux)
    message(STATUS "Using mock i2c ${gcimock_BINARY_DIR}")
    target_link_directories(${PROJECT_NAME}
        PUBLIC
            ${gcimock_BINARY_DIR}
            # ${squaternion_BINARY_DIR}
    )
    target_include_directories(${PROJECT_NAME}
        PUBLIC
            ${gcimock_SOURCE_DIR}/src
            # ${squaternion_SOURCE_DIR}/src
    )
    # target_link_libraries(${PROJECT_NAME}
    #     PRIVATE
    #         gciMock
    #         squaternion
    # )
endif()

# Examples ====================================================================
# option(BUILD_EXAMPLES "Examples" ON)
message(STATUS "Building gciSensors examples is ${BUILD_EXAMPLES}")
if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# option(BUILD_GTESTS "build gtests" ON)
message(STATUS "Building gciSensors gtests is ${BUILD_GTESTS}")
if (BUILD_GTESTS)
    add_subdirectory(gtests)
endif()

