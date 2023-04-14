#pragma once

#include <sensor_msgs/Imu.h>

#include <mutex>
#include <queue>
#include <optional>

namespace ORB_SLAM3 {
  namespace IMU {
    struct Point;
  }
}  // namespace ORB_SLAM3

namespace ros {
  struct NodeHandle;
  struct Subscriber;
}  // namespace ros

namespace orbslam3_ros_docker {
  struct config_t;

  class ImuCallbackHandler {
  private:
    std::mutex mutable _mutex;
    std::queue<sensor_msgs::ImuConstPtr> _buffer;
    std::unique_ptr<ros::Subscriber> __subscriber__;

    void onImu(sensor_msgs::ImuConstPtr const& msg);

  public:
    ImuCallbackHandler(
      ros::NodeHandle& node, std::shared_ptr<config_t const> config);
    ~ImuCallbackHandler();

    std::optional<std::vector<ORB_SLAM3::IMU::Point>> popDataUntilTimestamp(
      double timestamp);
  };
}  // namespace orbslam3_ros_docker
