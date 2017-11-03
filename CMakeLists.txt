cmake_minimum_required(VERSION 2.8.3)
project(folding_assembly_controller)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  cmake_modules
  eigen_conversions
  message_generation
  std_msgs
	actionlib
	actionlib_msgs
	kdl_parser
	sensor_msgs
	tf_conversions
	tf
	abb_irb14000_support
  sarafun_msgs
  generic_control_toolbox
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
	AdaptiveController.msg
)

add_action_files(
	DIRECTORY action
	FILES FoldingController.action
)

generate_messages(
  DEPENDENCIES
	actionlib_msgs
  geometry_msgs
  generic_control_toolbox
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs generic_control_toolbox
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(
  dependencies
  src/kalman_filter.cpp
  src/ects.cpp
  src/adaptive_velocity_controller.cpp
  src/pose_controller.cpp
)

add_executable(folding_action_node src/folding_action_node.cpp ${dependencies})
target_link_libraries(folding_action_node ${catkin_LIBRARIES})
add_dependencies(folding_action_node ${catkin_EXPORTED_TARGETS})
