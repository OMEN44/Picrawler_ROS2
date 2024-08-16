cmake_minimum_required(VERSION 3.8)
project(picrawler_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(picrawler_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Digital pin node
# add_executable(picrawler_digital_pin src/picrawler_digital_pin.cpp)
# target_link_libraries(picrawler_digital_pin pigpiod_if2)
# ament_target_dependencies(picrawler_digital_pin rclcpp picrawler_interfaces)
# target_include_directories(picrawler_digital_pin PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# Robot hat service node
add_executable(picrawler_robot_hat src/picrawler_robot_hat.cpp)
target_link_libraries(picrawler_robot_hat pigpiod_if2)
ament_target_dependencies(picrawler_robot_hat std_msgs rclcpp picrawler_interfaces)
target_include_directories(picrawler_robot_hat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS
  # picrawler_digital_pin
  picrawler_robot_hat
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_export_dependencies(rclcpp)

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
