cmake_minimum_required(VERSION 3.10)
project(orbslam3_docker_ros)

find_package (catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   cv_bridge
   image_transport
   message_generation
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

catkin_package (
   CATKIN_DEPENDS
   roscpp std_msgs cv_bridge image_transport sensor_msgs message_runtime
)

add_executable(mono_imu "")
target_include_directories(
  mono_imu PRIVATE
  ${catkin_INCLUDE_DIRS}
  src
  # Since we are running inside a container, it is fine to hard-code the library
  # include paths.
  /usr/local/include/Thirdparty/Sophus
  /usr/local/include/CameraModels
)
target_link_libraries(
  mono_imu PRIVATE
  pangolin
  opencv_core
  Eigen3::Eigen
  ${catkin_LIBRARIES}

  /usr/local/lib/libORB_SLAM3.so
)
target_compile_features(mono_imu PRIVATE cxx_std_17)

add_subdirectory(src)

install(TARGETS mono_imu
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
