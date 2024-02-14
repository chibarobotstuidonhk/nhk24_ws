cmake_minimum_required(VERSION 3.8)
project(nhk24_use_amcl)

# # for debug
# execute_process(COMMAND ${CMAKE_CXX_COMPILER} --version OUTPUT_VARIABLE compiler_version_output)
# message(WARNING "Compiler version: ${compiler_version_output}")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20 -fconcepts-diagnostics-depth=2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(can_plugins2 REQUIRED)
find_package(nhk24_utils REQUIRED)

include_directories(
  include
)

add_executable(filter_node
  src/filter_node.cpp
)
ament_target_dependencies(filter_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nhk24_utils
)

add_executable(omni4_node
  src/omni4.cpp
)
ament_target_dependencies(omni4_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  can_plugins2
  nhk24_utils
)

add_executable(genbacat_node
  src/genbacat_node.cpp
)
ament_target_dependencies(genbacat_node
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2_ros
  nhk24_utils
)

add_executable(pacman_node
  src/pacman.cpp
)
ament_target_dependencies(pacman_node
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  nhk24_utils
)

add_executable(path_loader_node
  src/path_loader.cpp
)
ament_target_dependencies(path_loader_node
  rclcpp
  rclcpp_components
  std_msgs
  nhk24_utils
)

install(
  TARGETS
    filter_node
    omni4_node
    pacman_node
    path_loader_node
    genbacat_node
  DESTINATION lib/${PROJECT_NAME}
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
