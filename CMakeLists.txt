cmake_minimum_required(VERSION 3.0.2)
project(orbslam3_docker)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)
catkin_package()

add_custom_target(
  orbslam3_docker_build_container
  ALL COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/build)
