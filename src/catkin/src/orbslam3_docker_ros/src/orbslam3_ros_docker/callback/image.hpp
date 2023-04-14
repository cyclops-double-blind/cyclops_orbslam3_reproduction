#pragma once

#include <sensor_msgs/Image.h>

#include <memory>
#include <thread>
#include <string>

namespace ORB_SLAM3 {
  struct System;
}

namespace ros {
  struct NodeHandle;
}

namespace orbslam3_ros_docker {
  struct config_t;
  struct ImuCallbackHandler;

  class ImageCallbackHandler {
  private:
    struct Impl;
    std::unique_ptr<Impl> _pimpl;

  public:
    ImageCallbackHandler(
      ros::NodeHandle& node, std::shared_ptr<config_t const> config,
      std::unique_ptr<ORB_SLAM3::System> slam,
      std::unique_ptr<ImuCallbackHandler> imu_grabber);
    ~ImageCallbackHandler();

    std::thread startDataConsumeThread();
  };
}  // namespace orbslam3_ros_docker
