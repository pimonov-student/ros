cmake_minimum_required(VERSION 3.5)

project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(run src/circle_movement.cpp)
ament_target_dependencies(run rclcpp geometry_msgs)

# Исполняемый файл run
install (
  TARGETS
  	run
  DESTINATION
  	lib/${PROJECT_NAME})

# Папка с launch файлами
install(
  DIRECTORY
    launch/
  DESTINATION
  	share/${PROJECT_NAME}/launch
)

# Папка с конфигами
install(
  DIRECTORY
    config/
  DESTINATION
  	share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
