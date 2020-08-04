cmake_minimum_required(VERSION 3.0.2)
project(gazebo_selfie_sim)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

message(x ${CMAKE_INSTALL_PREFIX})
message(x ${CATKIN_DEVEL_PREFIX})
message(x ${CATKIN_PACKAGE_BIN_DESTINATION})
