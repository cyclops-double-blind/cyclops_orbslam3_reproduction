#pragma once

#include <Thirdparty/Sophus/sophus/se3.hpp>

#include <memory>
#include <vector>

namespace cv {
  struct Mat;
}

namespace tf2_ros {
  struct TransformBroadcaster;
}  // namespace tf2_ros

namespace ros {
  struct Time;
  struct NodeHandle;
  struct Publisher;
}  // namespace ros

namespace image_transport {
  struct Publisher;
}  // namespace image_transport

namespace ORB_SLAM3 {
  struct MapPoint;
}  // namespace ORB_SLAM3

namespace orbslam3_ros_docker {
  struct config_t;

  class TopicPublishHandler {
  private:
    std::shared_ptr<config_t const> _config;

    std::unique_ptr<ros::Publisher> _camera_pose_publisher;
    std::unique_ptr<ros::Publisher> _landmark_pointcloud_publisher;
    std::unique_ptr<image_transport::Publisher> _frame_image_publisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf2_broadcaster;

  public:
    explicit TopicPublishHandler(
      ros::NodeHandle& node, std::shared_ptr<config_t const> config);
    ~TopicPublishHandler();

    void publishCameraPose(
      ros::Time const& timestamp, Sophus::SE3f const& camera_pose);
    void publishCameraPoseTf2(
      ros::Time const& timestamp, Sophus::SE3f const& camera_pose);
    void publishLandmarkPointcloud(
      ros::Time const& timestamp,
      std::vector<ORB_SLAM3::MapPoint*> const& map_points);

    void publishTrackingFrameImage(
      ros::Time const& timestamp, cv::Mat const& image);
  };
}  // namespace orbslam3_ros_docker
