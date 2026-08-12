#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <deque>
#include <mutex>
#include <cmath>

namespace stereo_depth_yolo
{

class StereoHeaderSyncNode : public rclcpp::Node
{
public:
  explicit StereoHeaderSyncNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stereo_header_sync_node", options)
  {
    // Publishers
    l_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/synced/left/image_raw", 10);
    l_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/synced/left/camera_info", 10);
    r_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/synced/right/image_raw", 10);
    r_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/synced/right/camera_info", 10);

    // Subscribers requesting UniquePtr for true Zero-Copy Intra-Process ownership
    l_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/left/image_raw", 10,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        l_img_q_.push_back(std::move(msg)); enforce_max_size(); try_sync();
      });
    l_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/left/camera_info", 10,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        l_info_q_.push_back(std::move(msg)); enforce_max_size(); try_sync();
      });
    r_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/right/image_raw", 10,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        r_img_q_.push_back(std::move(msg)); enforce_max_size(); try_sync();
      });
    r_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/right/camera_info", 10,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        r_info_q_.push_back(std::move(msg)); enforce_max_size(); try_sync();
      });
  }

private:
  std::mutex sync_mutex_;
  std::deque<sensor_msgs::msg::Image::UniquePtr> l_img_q_, r_img_q_;
  std::deque<sensor_msgs::msg::CameraInfo::UniquePtr> l_info_q_, r_info_q_;

  void enforce_max_size() {
    // Prevents Out-Of-Memory crashes if a topic drops
    if (l_img_q_.size() > 15) l_img_q_.pop_front();
    if (l_info_q_.size() > 15) l_info_q_.pop_front();
    if (r_img_q_.size() > 15) r_img_q_.pop_front();
    if (r_info_q_.size() > 15) r_info_q_.pop_front();
  }

  void try_sync() {
    if (l_img_q_.empty() || l_info_q_.empty() || r_img_q_.empty() || r_info_q_.empty()) {
      return;
    }

    auto l_img_t = rclcpp::Time(l_img_q_.front()->header.stamp);
    auto l_info_t = rclcpp::Time(l_info_q_.front()->header.stamp);
    
    // Ensure Left camera image and info match
    if (l_img_t != l_info_t) {
      if (l_img_t < l_info_t) l_img_q_.pop_front(); else l_info_q_.pop_front();
      return;
    }

    auto r_img_t = rclcpp::Time(r_img_q_.front()->header.stamp);
    auto r_info_t = rclcpp::Time(r_info_q_.front()->header.stamp);
    
    // Ensure Right camera image and info match
    if (r_img_t != r_info_t) {
      if (r_img_t < r_info_t) r_img_q_.pop_front(); else r_info_q_.pop_front();
      return;
    }

    // Check approximate match between Left and Right cameras
    double diff = (l_img_t - r_img_t).seconds();
    
    if (std::abs(diff) <= 0.05) {  // 50ms tolerance
      // MATCH FOUND! Overwrite timestamps in place (No Deep Copy)
      auto target_stamp = l_img_q_.front()->header.stamp;
      l_info_q_.front()->header.stamp = target_stamp;
      r_img_q_.front()->header.stamp = target_stamp;
      r_info_q_.front()->header.stamp = target_stamp;

      // Transfer memory ownership directly to the publishers
      l_img_pub_->publish(std::move(l_img_q_.front()));
      l_info_pub_->publish(std::move(l_info_q_.front()));
      r_img_pub_->publish(std::move(r_img_q_.front()));
      r_info_pub_->publish(std::move(r_info_q_.front()));

      l_img_q_.pop_front(); l_info_q_.pop_front();
      r_img_q_.pop_front(); r_info_q_.pop_front();
    } 
    else if (diff > 0.05) {
      // Right stream is lagging
      r_img_q_.pop_front(); r_info_q_.pop_front();
    } 
    else {
      // Left stream is lagging
      l_img_q_.pop_front(); l_info_q_.pop_front();
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr l_img_pub_, r_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr l_info_pub_, r_info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr l_img_sub_, r_img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr l_info_sub_, r_info_sub_;
};

}  // namespace stereo_depth_yolo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_depth_yolo::StereoHeaderSyncNode)