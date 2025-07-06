#include "orbslam3_ros_docker/publish/publisher.hpp"
#include "orbslam3_ros_docker/config.hpp"

#include "MapPoint.h"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/node_handle.h>

namespace orbslam3_ros_docker {
  TopicPublishHandler::TopicPublishHandler(
    ros::NodeHandle& node, std::shared_ptr<config_t const> config)
      : _config(config) {
    _camera_pose_publisher = std::make_unique<ros::Publisher>(
      node.advertise<geometry_msgs::PoseStamped>(
        config->camera_pose_publish_topic,
        config->camera_pose_publish_queue_size));
    _landmark_pointcloud_publisher =
      std::make_unique<ros::Publisher>(node.advertise<sensor_msgs::PointCloud2>(
        config->landmark_pointcloud_publish_topic,
        config->landmark_pointcloud_publish_queue_size));

    image_transport::ImageTransport image_transport(node);
    _frame_image_publisher =
      std::make_unique<image_transport::Publisher>(image_transport.advertise(
        config->tracking_image_publish_topic,
        config->tracking_image_publish_queue_size));

    _tf2_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
  }

  TopicPublishHandler::~TopicPublishHandler() = default;

  template <typename vec3_msg_t>
  static vec3_msg_t make_vec3_msg(Eigen::Vector3f const& v) {
    vec3_msg_t msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
  }

  static geometry_msgs::Quaternion make_quaternion_msg(
    Eigen::Quaternionf const& q) {
    geometry_msgs::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
  }

  void TopicPublishHandler::publishCameraPose(
    ros::Time const& timestamp, Sophus::SE3f const& x) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = _config->world_frame_id;
    pose_msg.header.stamp = timestamp;

    pose_msg.pose.position =
      make_vec3_msg<geometry_msgs::Point>(x.translation());
    pose_msg.pose.orientation = make_quaternion_msg(x.unit_quaternion());
    _camera_pose_publisher->publish(pose_msg);
  }

  void TopicPublishHandler::publishCameraPoseTf2(
    ros::Time const& timestamp, Sophus::SE3f const& x) {
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = _config->world_frame_id;
    msg.child_frame_id = _config->camera_frame_id;
    msg.header.stamp = timestamp;

    msg.transform.translation =
      make_vec3_msg<geometry_msgs::Vector3>(x.translation());
    msg.transform.rotation = make_quaternion_msg(x.unit_quaternion());
    _tf2_broadcaster->sendTransform(msg);
  }

  static sensor_msgs::PointCloud2 make_pointcloud2_message(
    ros::Time const& timestamp, std::string const& world_frame_id,
    std::vector<ORB_SLAM3::MapPoint*> const& map_points) {
    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = timestamp;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 3 * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(3);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < 3; i++) {
      cloud.fields[i].name = channel_id[i];
      cloud.fields[i].offset = i * sizeof(float);
      cloud.fields[i].count = 1;
      cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);
    for (int i = 0; i < cloud.width; i++) {
      auto map_point = map_points.at(i);
      if (map_point == nullptr)
        continue;

      auto f = map_point->GetWorldPos();
      float buffer[3] = {f.x(), f.y(), f.z()};
      memcpy(
        cloud.data.data() + (i * cloud.point_step), buffer, 3 * sizeof(float));
    }
    return cloud;
  }

  void TopicPublishHandler::publishLandmarkPointcloud(
    ros::Time const& timestamp,
    std::vector<ORB_SLAM3::MapPoint*> const& map_points) {
    auto cloud =
      make_pointcloud2_message(timestamp, _config->world_frame_id, map_points);
    _landmark_pointcloud_publisher->publish(cloud);
  }

  void TopicPublishHandler::publishTrackingFrameImage(
    ros::Time const& timestamp, cv::Mat const& image) {
    auto header = std_msgs::Header();
    header.frame_id = _config->camera_frame_id;
    header.stamp = timestamp;

    auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    _frame_image_publisher->publish(msg);
  }
}  // namespace orbslam3_ros_docker
