#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace stereo_depth_yolo
{

class StereoHeaderSyncNode : public rclcpp::Node
{
public:
  explicit StereoHeaderSyncNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stereo_header_sync_node", options)
  {
    left_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/synced/left/image_raw", 10);
    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/synced/left/camera_info", 10);
    right_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/synced/right/image_raw", 10);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/synced/right/camera_info", 10);

    left_img_sub_.subscribe(this, "/left/image_raw");
    left_info_sub_.subscribe(this, "/left/camera_info");
    right_img_sub_.subscribe(this, "/right/image_raw");
    right_info_sub_.subscribe(this, "/right/camera_info");

    sync_ = std::make_unique<message_filters::Synchronizer<ApproxSyncPolicy>>(
      ApproxSyncPolicy(10), left_img_sub_, left_info_sub_, right_img_sub_, right_info_sub_);
    
    sync_->setAgePenalty(0.02);
    sync_->registerCallback(std::bind(&StereoHeaderSyncNode::syncCallback, this, 
                            std::placeholders::_1, std::placeholders::_2, 
                            std::placeholders::_3, std::placeholders::_4));
  }

private:
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & left_info,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & right_info)
  {
    sensor_msgs::msg::Image out_left_img = *left_img;
    sensor_msgs::msg::CameraInfo out_left_info = *left_info;
    sensor_msgs::msg::Image out_right_img = *right_img;
    sensor_msgs::msg::CameraInfo out_right_info = *right_info;

    auto unified_stamp = left_img->header.stamp;
    out_left_img.header.stamp = unified_stamp;
    out_left_info.header.stamp = unified_stamp;
    out_right_img.header.stamp = unified_stamp;
    out_right_info.header.stamp = unified_stamp;

    left_img_pub_->publish(out_left_img);
    left_info_pub_->publish(out_left_info);
    right_img_pub_->publish(out_right_img);
    right_info_pub_->publish(out_right_info);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

  message_filters::Subscriber<sensor_msgs::msg::Image> left_img_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_img_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_info_sub_;

  std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;
};

}  // namespace stereo_depth_yolo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_depth_yolo::StereoHeaderSyncNode)
