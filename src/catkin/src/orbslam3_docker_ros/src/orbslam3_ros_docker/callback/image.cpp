#include "orbslam3_ros_docker/callback/image.hpp"
#include "orbslam3_ros_docker/callback/imu.hpp"
#include "orbslam3_ros_docker/config.hpp"
#include "orbslam3_ros_docker/publish/publisher.hpp"

#include "System.h"  // ORB-SLAM3 header. (provided without project subdirectory)

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <ros/node_handle.h>

#include <std_msgs/Bool.h>

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
    std::unique_ptr<TopicPublishHandler> _topic_publisher;

    ros::Subscriber __subscriber__;
    ros::Subscriber __reset_subscriber__;

    std::mutex mutable _mutex;
    bool _reset_request = false;
    std::queue<sensor_msgs::ImageConstPtr> _buffer;

    cv::Ptr<cv::CLAHE> _clahe;

    void dataConsumeJob();
    void onImage(sensor_msgs::ImageConstPtr const& msg);
    void onReset(std_msgs::BoolConstPtr const& msg);

    bool popReset();
    optional<tuple<ros::Time, cv::Mat, unique_lock_t>> grabImage() const;
    unique_lock_t popImage(unique_lock_t grab_lock);

  public:
    Impl(
      ros::NodeHandle& node, std::shared_ptr<config_t const> config,
      std::unique_ptr<ORB_SLAM3::System> slam,
      std::unique_ptr<ImuCallbackHandler> imu_grabber,
      std::unique_ptr<TopicPublishHandler> topic_publisher);
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

  void ImageCallbackHandler::Impl::onReset(std_msgs::BoolConstPtr const& msg) {
    std::lock_guard<std::mutex> _(_mutex);
    _reset_request = true;
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

  bool ImageCallbackHandler::Impl::popReset() {
    std::lock_guard<std::mutex> _(_mutex);
    auto result = _reset_request;
    _reset_request = false;
    return result;
  }

  optional<tuple<ros::Time, cv::Mat, unique_lock_t>>
  ImageCallbackHandler::Impl::grabImage() const {
    std::unique_lock<std::mutex> grab_lock(_mutex);

    if (_buffer.empty())
      return std::nullopt;

    auto const& msg = _buffer.front();
    auto timestamp = msg->header.stamp;
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
      auto& [image_rostime, image, lock] = *maybe_image;
      auto image_timestamp = image_rostime.toSec();

      auto imu_frame = _imu_grabber->popDataUntilTimestamp(image_timestamp);
      if (!imu_frame)
        continue;
      popImage(std::move(lock));

      if (imu_frame->empty())
        continue;

      _clahe->apply(image, image);
      auto x_cw = _slam->TrackMonocular(image, image_timestamp, *imu_frame);

      _topic_publisher->publishTrackingFrameImage(
        image_rostime, _slam->GetTrackingFrameImage());
      if (_slam->isIMUInitialized()) {
        auto x_wc = x_cw.inverse();

        _topic_publisher->publishCameraPose(image_rostime, x_wc);
        _topic_publisher->publishCameraPoseTf2(image_rostime, x_wc);
        _topic_publisher->publishLandmarkPointcloud(
          image_rostime, _slam->GetTrackedMapPoints());

        if (popReset()) {
          _slam->Reset();
          {
            std::lock_guard<std::mutex> _(_mutex);
            _buffer = {};
          }
          _imu_grabber->clear();
        }
      }
    }
  }

  std::thread ImageCallbackHandler::Impl::startDataConsumeThread() {
    return std::thread(&Impl::dataConsumeJob, this);
  }

  ImageCallbackHandler::Impl::Impl(
    ros::NodeHandle& node, std::shared_ptr<config_t const> config,
    std::unique_ptr<ORB_SLAM3::System> slam,
    std::unique_ptr<ImuCallbackHandler> imu_grabber,
    std::unique_ptr<TopicPublishHandler> topic_publisher)
      : _config(config),
        _slam(std::move(slam)),
        _imu_grabber(std::move(imu_grabber)),
        _topic_publisher(std::move(topic_publisher)) {
    __subscriber__ = node.subscribe(
      config->image_topic, config->image_callback_queue_size, &Impl::onImage,
      this);
    __reset_subscriber__ = node.subscribe("reset", 1, &Impl::onReset, this);

    if (config->clahe_config) {
      auto const& clahe_config = *config->clahe_config;
      auto const& [w, h] = clahe_config.tilesize;
      _clahe = cv::createCLAHE(clahe_config.clip_limit, cv::Size(w, h));
    }
  }

  ImageCallbackHandler::ImageCallbackHandler(
    ros::NodeHandle& node, std::shared_ptr<config_t const> config,
    std::unique_ptr<ORB_SLAM3::System> slam,
    std::unique_ptr<ImuCallbackHandler> imu_grabber,
    std::unique_ptr<TopicPublishHandler> topic_publisher)
      : _pimpl(std::make_unique<Impl>(
          node, config, std::move(slam), std::move(imu_grabber),
          std::move(topic_publisher))) {
  }

  ImageCallbackHandler::~ImageCallbackHandler() = default;

  std::thread ImageCallbackHandler::startDataConsumeThread() {
    return _pimpl->startDataConsumeThread();
  }
}  // namespace orbslam3_ros_docker
