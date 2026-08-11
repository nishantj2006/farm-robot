#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// Isaac ROS NITROS Includes
#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_builder.hpp"

#include <cuda_runtime.h>

class V4l2NitrosCamera : public rclcpp::Node {
public:
    V4l2NitrosCamera() : Node("v4l2_nitros_camera") {
        
        // 1. Initialize Left and Right Managed NITROS Publishers
        left_nitros_pub_ = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>(
            this, "/left/image_raw", nvidia::isaac_ros::nitros::nitros_image_bgr8_t::supported_type_name);
            
        right_nitros_pub_ = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>(
            this, "/right/image_raw", nvidia::isaac_ros::nitros::nitros_image_bgr8_t::supported_type_name);

        // 2. Setup standard ROS publishers for CameraInfo
        left_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/left/camera_info", 10);
        right_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/right/camera_info", 10);

        RCLCPP_INFO(this->get_logger(), "Stereo NITROS Camera Node Initialized.");
    }

    // Call this function when your V4L2 logic decodes a side-by-side frame
    // Note: removed 'stride' parameter to resolve unused variable warning
    void publish_stereo_gpu_frames(void* d_left_cuda_buffer, void* d_right_cuda_buffer, uint32_t single_width, uint32_t height) {
        
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_link";

        // 3. Build Left GPU Image
        nvidia::isaac_ros::nitros::NitrosImageBuilder left_builder;
        nvidia::isaac_ros::nitros::NitrosImage left_nitros_image = left_builder.WithHeader(header)
               .WithDimensions(single_width, height)
               .WithEncoding("bgr8")
               .WithGpuData(d_left_cuda_buffer)
               .Build();
               
        // 4. Build Right GPU Image
        nvidia::isaac_ros::nitros::NitrosImageBuilder right_builder;
        nvidia::isaac_ros::nitros::NitrosImage right_nitros_image = right_builder.WithHeader(header)
               .WithDimensions(single_width, height)
               .WithEncoding("bgr8")
               .WithGpuData(d_right_cuda_buffer)
               .Build();
               
        // 5. Publish Directly to Isaac ROS Graph (Zero CPU overhead)
        left_nitros_pub_->publish(left_nitros_image);
        right_nitros_pub_->publish(right_nitros_image);
        
        // Note: Remember to populate and publish CameraInfo logic here as well
    }

private:
    std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>> left_nitros_pub_;
    std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>> right_nitros_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<V4l2NitrosCamera>());
    rclcpp::shutdown();
    return 0;
}
