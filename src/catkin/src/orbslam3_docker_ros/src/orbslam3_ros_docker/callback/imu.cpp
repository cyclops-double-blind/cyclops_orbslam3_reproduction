#include "orbslam3_ros_docker/callback/imu.hpp"
#include "orbslam3_ros_docker/config.hpp"
#include "ImuTypes.h"  // ORB-SLAM3 header. (provided without project subdirectory)

#include <ros/node_handle.h>

namespace orbslam3_ros_docker {
  ImuCallbackHandler::ImuCallbackHandler(
    ros::NodeHandle& node, std::shared_ptr<config_t const> config) {
    __subscriber__ = std::make_unique<ros::Subscriber>(node.subscribe(
      config->imu_topic, config->imu_callback_queue_size,
      &ImuCallbackHandler::onImu, this));
  }

  ImuCallbackHandler::~ImuCallbackHandler() = default;

  void ImuCallbackHandler::onImu(sensor_msgs::ImuConstPtr const& msg) {
    std::lock_guard<std::mutex> _(_mutex);
    _buffer.push(msg);
    return;
  }

  std::optional<std::vector<ORB_SLAM3::IMU::Point>>
  ImuCallbackHandler::popDataUntilTimestamp(double timestamp) {
    std::lock_guard<std::mutex> _(_mutex);

    if (timestamp > _buffer.back()->header.stamp.toSec())
      return std::nullopt;

    std::vector<ORB_SLAM3::IMU::Point> result;
    while (!_buffer.empty()) {
      auto const& msg = _buffer.front();
      double msgtime = msg->header.stamp.toSec();
      if (msgtime > timestamp)
        break;

      auto const& a = msg->linear_acceleration;
      auto const& w = msg->angular_velocity;
      result.push_back(ORB_SLAM3::IMU::Point(
        cv::Point3f(a.x, a.y, a.z), cv::Point3f(w.x, w.y, w.z), msgtime));
      _buffer.pop();
    }
    return result;
  }
}  // namespace orbslam3_ros_docker
