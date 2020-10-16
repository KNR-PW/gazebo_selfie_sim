cmake_minimum_required(VERSION 3.1.2)
project(gazebo_selfie_sim)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  custom_msgs
  gazebo_ros
  roslint
  robot_state_publisher
  xacro
  joint_state_publisher
  effort_controllers
  controller_manager
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED )

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS custom_msgs
)

roslint_cpp()
roslint_python()

add_library(IndicatorPlugin src/IndicatorPlugin.cpp)

target_link_libraries(IndicatorPlugin
${catkin_LIBRARIES} 
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
)
add_dependencies(IndicatorPlugin ${catkin_EXPORTED_TARGETS})


install(PROGRAMS
  scripts/manual_steering.py
  scripts/simulation_bridge.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
