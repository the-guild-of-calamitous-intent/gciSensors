cmake_minimum_required(VERSION 3.8)
project(gci_sensors)

# Default to C17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include(FetchContent)

# squaterion ------------
FetchContent_Declare(squaternion
    GIT_REPOSITORY "https://github.com/MomsFriendlyRobotCompany/squaternion.git"
    GIT_TAG "origin/master"
    SOURCE_SUBDIR "cpp"
)

FetchContent_MakeAvailable( squaternion )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(SDK_SRC "${CMAKE_SOURCE_DIR}/..")

add_executable(main
    "${CMAKE_SOURCE_DIR}/src/main.cpp")
target_include_directories(main PRIVATE
    "${SDK_SRC}/linux/"
    "${SDK_SRC}/src/")
target_link_libraries(main squaternion)

ament_target_dependencies(main rclcpp std_msgs sensor_msgs geometry_msgs)

# you NEED this so setup.bash can find the binary
install(
  TARGETS
    main
  DESTINATION
    "lib/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
