#include "orbslam3_ros_docker/config.hpp"
#include <ros/node_handle.h>

#include <optional>

namespace orbslam3_ros_docker {
  template <typename value_t>
  static std::optional<value_t> maybe_get_param(
    ros::NodeHandle& node, std::string const& name) {
    value_t value;
    if (node.getParam(name, value))
      return value;
    return std::nullopt;
  }

  template <typename value_t>
  static value_t get_param(
    ros::NodeHandle& node, std::string const& name, value_t const& _default) {
    value_t value;
    if (node.getParam(name, value))
      return value;
    return _default;
  }

  static clahe_config_t make_clahe_config(
    std::optional<double> clip_limit, std::optional<int> tilesize_width,
    std::optional<int> tilesize_height) {
    return clahe_config_t {
      .clip_limit = clip_limit ? *clip_limit : 3.0,
      .tilesize = std::make_tuple(
        tilesize_width ? *tilesize_width : 8,
        tilesize_height ? *tilesize_height : 8),
    };
  }

  static std::optional<clahe_config_t> read_clahe_config(
    ros::NodeHandle& node) {
    auto clahe_enable = maybe_get_param<bool>(node, "clahe/enable");
    auto clip_limit = maybe_get_param<double>(node, "clahe/clip_limit");
    auto tilesize_width = maybe_get_param<int>(node, "clahe/tilesize/width");
    auto tilesize_height = maybe_get_param<int>(node, "clahe/tilesize/height");

    if (!clahe_enable) {
      if (!clip_limit && !tilesize_width && !tilesize_height)
        return std::nullopt;
      return make_clahe_config(clip_limit, tilesize_width, tilesize_height);
    }

    if (*clahe_enable == false)
      return std::nullopt;
    return make_clahe_config(clip_limit, tilesize_width, tilesize_height);
  }

  std::unique_ptr<config_t const> readconfig(ros::NodeHandle& node) {
    auto maybe_vocabulary_path =
      maybe_get_param<std::string>(node, "vocabulary_path");
    if (!maybe_vocabulary_path) {
      ROS_ERROR("config: vocabulary file path is not given.");
      return nullptr;
    }
    auto maybe_settings_path =
      maybe_get_param<std::string>(node, "settings_path");
    if (!maybe_settings_path) {
      ROS_ERROR("config: settings file path is not given.");
      return nullptr;
    }

    return std::make_unique<config_t>(config_t {
      .clahe_config = read_clahe_config(node),
      .imu_callback_queue_size =
        get_param<int>(node, "imu_callback_queue_size", 1024),
      .image_callback_queue_size =
        get_param<int>(node, "image_callback_queue_size", 256),
      .image_data_buffer_maxsize =
        get_param<int>(node, "image_data_buffer_maxsize", 128),

      .tracking_image_publish_queue_size =
        get_param<int>(node, "tracking_image_publish_queue_size", 8),
      .camera_pose_publish_queue_size =
        get_param<int>(node, "camera_pose_publish_queue_size", 32),
      .landmark_pointcloud_publish_queue_size =
        get_param<int>(node, "landmark_pointcloud_publish_queue_size", 1),

      .data_consume_worker_loop_delay_ms =
        get_param<int>(node, "data_consume_worker_loop_delay_ms", 1),

      .imu_topic = get_param<std::string>(node, "imu_topic", "imu"),
      .image_topic =
        get_param<std::string>(node, "image_topic", "camera/image_raw"),

      .tracking_image_publish_topic = get_param<std::string>(
        node, "tracking_image_publish_topic", "tracking_image"),
      .camera_pose_publish_topic = get_param<std::string>(
        node, "camera_pose_publish_topic", "camera_pose"),
      .landmark_pointcloud_publish_topic =
        get_param<std::string>(node, "map_point_publish_topic", "map_points"),

      .vocabulary_path = *maybe_vocabulary_path,
      .settings_path = *maybe_settings_path,

      .world_frame_id = get_param<std::string>(node, "world_frame_id", "map"),
      .camera_frame_id =
        get_param<std::string>(node, "camera_frame_id", "camera"),
    });
  }
}  // namespace orbslam3_ros_docker
