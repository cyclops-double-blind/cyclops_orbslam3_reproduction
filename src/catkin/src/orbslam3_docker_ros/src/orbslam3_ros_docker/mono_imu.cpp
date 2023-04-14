#include "orbslam3_ros_docker/callback/image.hpp"
#include "orbslam3_ros_docker/callback/imu.hpp"
#include "orbslam3_ros_docker/config.hpp"

#include "System.h"  // ORB-SLAM3 header. (provided without project subdirectory)

#include <ros/ros.h>

using orbslam3_ros_docker::ImageCallbackHandler;
using orbslam3_ros_docker::ImuCallbackHandler;

int main(int argc, char** argv) {
  ros::init(argc, argv, "orbslam3_ros_mono_imu_docker");
  ros::NodeHandle node, pnode("~");

  ros::console::set_logger_level(
    ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  std::shared_ptr config = orbslam3_ros_docker::readconfig(pnode);

  if (config == nullptr) {
    ROS_ERROR("Failed to read config");
    return 1;
  }

  auto orbslam = std::make_unique<ORB_SLAM3::System>(
    config->vocabulary_path, config->settings_path,
    ORB_SLAM3::System::IMU_MONOCULAR, false);

  auto imu_handler = std::make_unique<ImuCallbackHandler>(node, config);
  auto image_handler = std::make_unique<ImageCallbackHandler>(
    node, config, std::move(orbslam), std::move(imu_handler));

  auto consume_worker = image_handler->startDataConsumeThread();
  ros::spin();
  consume_worker.join();

  return 0;
}
