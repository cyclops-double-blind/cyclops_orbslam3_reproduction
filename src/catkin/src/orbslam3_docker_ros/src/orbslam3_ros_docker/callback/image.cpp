#include "orbslam3_ros_docker/callback/image.hpp"
#include "orbslam3_ros_docker/callback/imu.hpp"
#include "orbslam3_ros_docker/config.hpp"

#include "System.h"  // ORB-SLAM3 header. (provided without project subdirectory)

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <ros/node_handle.h>

#include <mutex>
#include <queue>
#include <vector>

#include <optional>
#include <tuple>

namespace orbslam3_ros_docker {
  using std::optional;
  using std::tuple;
  using std::unique_lock;

  using timestamp_t = double;

  using unique_lock_t = std::unique_lock<std::mutex>;

  class ImageCallbackHandler::Impl {
  private:
    std::shared_ptr<config_t const> _config;
    std::unique_ptr<ORB_SLAM3::System> _slam;
    std::unique_ptr<ImuCallbackHandler> _imu_grabber;
    ros::Subscriber __subscriber__;

    std::mutex mutable _mutex;
    std::queue<sensor_msgs::ImageConstPtr> _buffer;
    cv::Ptr<cv::CLAHE> _clahe;

    void dataConsumeJob();
    void onImage(sensor_msgs::ImageConstPtr const& msg);

    optional<tuple<double, cv::Mat, unique_lock_t>> grabImage() const;
    unique_lock_t popImage(unique_lock_t grab_lock);

  public:
    Impl(
      ros::NodeHandle& node, std::shared_ptr<config_t const> config,
      std::unique_ptr<ORB_SLAM3::System> slam,
      std::unique_ptr<ImuCallbackHandler> imu_grabber);
    std::thread startDataConsumeThread();
  };

  void ImageCallbackHandler::Impl::onImage(
    sensor_msgs::ImageConstPtr const& msg) {
    if (msg == nullptr)
      return;

    std::lock_guard<std::mutex> _(_mutex);
    while (_buffer.size() > _config->image_data_buffer_maxsize)
      _buffer.pop();
    _buffer.push(msg);
  }

  static optional<cv::Mat> make_cvimage(sensor_msgs::ImageConstPtr const& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return std::nullopt;
    }

    if (cv_ptr->image.type() != 0) {
      ROS_ERROR(
        "Image type error. expected: %d, got: %d", 0, cv_ptr->image.type());
    }
    return cv_ptr->image.clone();
  }

  optional<tuple<timestamp_t, cv::Mat, unique_lock_t>>
  ImageCallbackHandler::Impl::grabImage() const {
    std::unique_lock<std::mutex> grab_lock(_mutex);

    if (_buffer.empty())
      return std::nullopt;

    auto const& msg = _buffer.front();
    auto timestamp = msg->header.stamp.toSec();
    auto maybe_image = make_cvimage(msg);
    if (!maybe_image)
      return std::nullopt;
    return std::make_tuple(timestamp, *maybe_image, std::move(grab_lock));
  }

  unique_lock_t ImageCallbackHandler::Impl::popImage(unique_lock_t grab_lock) {
    _buffer.pop();
    return grab_lock;
  }

  void ImageCallbackHandler::Impl::dataConsumeJob() {
    auto loop_delay = _config->data_consume_worker_loop_delay_ms;
    while (ros::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(loop_delay));

      auto maybe_image = grabImage();
      if (!maybe_image)
        continue;
      auto& [image_timestamp, image, lock] = *maybe_image;

      auto imu_frame = _imu_grabber->popDataUntilTimestamp(image_timestamp);
      if (!imu_frame)
        continue;
      popImage(std::move(lock));

      if (imu_frame->empty())
        continue;

      _clahe->apply(image, image);
      _slam->TrackMonocular(image, image_timestamp, *imu_frame);

    }
  }

  std::thread ImageCallbackHandler::Impl::startDataConsumeThread() {
    return std::thread(&Impl::dataConsumeJob, this);
  }

  ImageCallbackHandler::Impl::Impl(
    ros::NodeHandle& node, std::shared_ptr<config_t const> config,
    std::unique_ptr<ORB_SLAM3::System> slam,
    std::unique_ptr<ImuCallbackHandler> imu_grabber)
      : _config(config),
        _slam(std::move(slam)),
        _imu_grabber(std::move(imu_grabber)) {
    __subscriber__ = node.subscribe(
      config->image_topic, config->image_callback_queue_size, &Impl::onImage,
      this);

    if (config->clahe_config) {
      auto const& clahe_config = *config->clahe_config;
      auto const& [w, h] = clahe_config.tilesize;
      _clahe = cv::createCLAHE(clahe_config.clip_limit, cv::Size(w, h));
    }
  }

  ImageCallbackHandler::ImageCallbackHandler(
    ros::NodeHandle& node, std::shared_ptr<config_t const> config,
    std::unique_ptr<ORB_SLAM3::System> slam,
    std::unique_ptr<ImuCallbackHandler> imu_grabber)
      : _pimpl(std::make_unique<Impl>(
          node, config, std::move(slam), std::move(imu_grabber))) {
  }

  ImageCallbackHandler::~ImageCallbackHandler() = default;

  std::thread ImageCallbackHandler::startDataConsumeThread() {
    return _pimpl->startDataConsumeThread();
  }
}  // namespace orbslam3_ros_docker
