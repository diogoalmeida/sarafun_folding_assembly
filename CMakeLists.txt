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
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  monitorMsg.msg
)

add_action_files(
	DIRECTORY action
	FILES FoldingAssembly.action Estimate.action
)

generate_messages(
  DEPENDENCIES
	actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(kalman_filter src/kalman_filter.cpp src/utils/matrix_parser.cpp)
target_link_libraries(kalman_filter ${catkin_LIBRARIES})
add_dependencies(kalman_filter ${catkin_EXPORTED_TARGETS})

add_library(ects_controller src/ects.cpp )
target_link_libraries(ects_controller ${catkin_LIBRARIES})
add_dependencies(ects_controller ${catkin_EXPORTED_TARGETS})

add_library(adaptive_controller src/adaptive_velocity_controller.cpp )
target_link_libraries(adaptive_controller ${catkin_LIBRARIES})
add_dependencies(adaptive_controller ${catkin_EXPORTED_TARGETS})

add_library(pose_controller src/pose_controller.cpp )
target_link_libraries(pose_controller ${catkin_LIBRARIES})
add_dependencies(pose_controller ${catkin_EXPORTED_TARGETS})
