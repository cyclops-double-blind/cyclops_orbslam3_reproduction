#pragma once

#include <memory>
#include <tuple>
#include <optional>

namespace ros {
  struct NodeHandle;
}

namespace orbslam3_ros_docker {
  struct clahe_config_t {
    double clip_limit;
    std::tuple<int, int> tilesize;
  };

  struct config_t {
    std::optional<clahe_config_t> clahe_config;

    int imu_callback_queue_size;
    int image_callback_queue_size;
    int image_data_buffer_maxsize;

    int tracking_image_publish_queue_size;
    int camera_pose_publish_queue_size;
    int landmark_pointcloud_publish_queue_size;

    int data_consume_worker_loop_delay_ms;

    std::string imu_topic;
    std::string image_topic;

    std::string tracking_image_publish_topic;
    std::string camera_pose_publish_topic;
    std::string landmark_pointcloud_publish_topic;

    std::string vocabulary_path;
    std::string settings_path;

    std::string world_frame_id;
    std::string camera_frame_id;
  };

  std::unique_ptr<config_t const> readconfig(ros::NodeHandle& node);
}  // namespace orbslam3_ros_docker
