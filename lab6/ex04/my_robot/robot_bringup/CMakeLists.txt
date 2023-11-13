cmake_minimum_required(VERSION 3.5)

project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(run src/circle_movement.cpp)
ament_target_dependencies(run rclcpp geometry_msgs)

add_executable(move src/movement.cpp)
ament_target_dependencies(move rclcpp 
						  geometry_msgs std_msgs nav_msgs 
						  tf2 tf2_geometry_msgs)

add_executable(move_to_wall src/move_to_wall.cpp)
ament_target_dependencies(move_to_wall rclcpp geometry_msgs sensor_msgs)

# Исполняемый файл run
install (
  TARGETS
  	run
  DESTINATION
  	lib/${PROJECT_NAME})

# Исполняемый файл move
install (
  TARGETS
  	move
  DESTINATION
  	lib/${PROJECT_NAME})

# Исполняемый файл move_to_wall
install (
  TARGETS
  	move_to_wall
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
