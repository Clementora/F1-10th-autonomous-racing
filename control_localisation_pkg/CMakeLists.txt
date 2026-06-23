cmake_minimum_required(VERSION 3.8)
project(control_localisation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(localiser_node src/localiser.cpp)
ament_target_dependencies(localiser_node rclcpp geometry_msgs sensor_msgs nav_msgs)


add_executable(control_node src/control.cpp)
ament_target_dependencies(control_node rclcpp sensor_msgs ackermann_msgs)

add_executable(disparity_extender src/disparity_extender.cpp)
ament_target_dependencies(disparity_extender rclcpp sensor_msgs ackermann_msgs nav_msgs)

install(TARGETS
  localiser_node
  control_node
  disparity_extender
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()