cmake_minimum_required(VERSION 3.8)
project(turtle_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(message_turtle_commands REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_library(action_server SHARED
	src/action_server.cpp)
target_include_directories(action_server PRIVATE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  	$<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
	PRIVATE "TURTLE_COMMANDS_BUILDING_DLL")
ament_target_dependencies(action_server
	"message_turtle_commands"
	"rclcpp"
	"rclcpp_action"
	"rclcpp_components"
	"geometry_msgs"
	"turtlesim")
rclcpp_components_register_node(action_server
	PLUGIN "turtle_action::TurtleActionServer"
	EXECUTABLE turtle_action_server)

add_library(action_client SHARED
	src/action_client.cpp)
target_include_directories(action_client PRIVATE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  	$<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
	PRIVATE "TURTLE_COMMANDS_BUILDING_DLL")
ament_target_dependencies(action_client
	"message_turtle_commands"
	"rclcpp"
	"rclcpp_action"
	"rclcpp_components")
rclcpp_components_register_node(action_client
	PLUGIN "turtle_action::TurtleActionClient"
	EXECUTABLE turtle_Server)

#add_executable(turtle_action_client src/action_client.cpp)
#ament_target_dependencies(turtle_action_client
#	message_turtle_commands
#	rclcpp
#	rclcpp_action
#	rclcpp_components)

install(TARGETS
	action_server
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)
install(TARGETS
	action_client
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)
#install(TARGETS
#	turtle_action_client
#	DESTINATION lib/${PROJECT_NAME})

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
