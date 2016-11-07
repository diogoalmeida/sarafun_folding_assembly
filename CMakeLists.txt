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
  #Eigen REQUIRED
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
  LIBRARIES folding_assembly_controller contact_point_estimator
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS}
)

add_library(folding_assembly_controller src/controller.cpp)
target_link_libraries(folding_assembly_controller ${catkin_LIBRARIES})# ${Eigen_LIBRARIES})

add_library(contact_point_estimator src/contact_point_estimator.cpp)
target_link_libraries(contact_point_estimator ${catkin_LIBRARIES})# ${Eigen_LIBRARIES})

add_executable(folding_assembly_node src/controller_node.cpp)
target_link_libraries(folding_assembly_node folding_assembly_controller contact_point_estimator ${catkin_LIBRARIES})
add_dependencies(folding_assembly_node ${catkin_EXPORTED_TARGETS})

add_executable(estimator_node src/estimator_node.cpp)
target_link_libraries(estimator_node contact_point_estimator ${catkin_LIBRARIES})
add_dependencies(estimator_node ${catkin_EXPORTED_TARGETS})
