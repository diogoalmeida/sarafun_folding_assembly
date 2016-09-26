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
  #Eigen REQUIRED
)

link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  monitorMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
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
