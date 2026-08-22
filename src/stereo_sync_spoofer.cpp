#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using ImageMsg = sensor_msgs::msg::Image;
using InfoMsg = sensor_msgs::msg::CameraInfo;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, InfoMsg, ImageMsg, InfoMsg>;

namespace stereo_sync_spoofer {

class StereoSyncSpoofer : public rclcpp::Node {
public:
    explicit StereoSyncSpoofer(const rclcpp::NodeOptions & options) 
        : Node("stereo_sync_spoofer", options), 
          left_in_count_(0), right_in_count_(0), synced_out_count_(0) {
        
        // Publishers for SLAM
        pub_img_l_ = this->create_publisher<ImageMsg>("/visual_slam/image_0", 10);
        pub_info_l_ = this->create_publisher<InfoMsg>("/visual_slam/camera_info_0", 10);
        pub_img_r_ = this->create_publisher<ImageMsg>("/visual_slam/image_1", 10);
        pub_info_r_ = this->create_publisher<InfoMsg>("/visual_slam/camera_info_1", 10);

        // Subscribers from Argus/Resize
        sub_img_l_.subscribe(this, "/left/image_raw");
        sub_info_l_.subscribe(this, "/left/camera_info");
        sub_img_r_.subscribe(this, "/right/image_raw");
        sub_info_r_.subscribe(this, "/right/camera_info");

        sub_img_l_.registerCallback([this](const ImageMsg::ConstSharedPtr&) { left_in_count_++; });
        sub_img_r_.registerCallback([this](const ImageMsg::ConstSharedPtr&) { right_in_count_++; });

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(30), sub_img_l_, sub_info_l_, sub_img_r_, sub_info_r_);
        
        sync_->registerCallback(std::bind(
            &StereoSyncSpoofer::sync_callback, this, 
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&StereoSyncSpoofer::log_stats, this));
            
        RCLCPP_INFO(this->get_logger(), "Stereo Sync Spoofer Composable Node active.");
    }

private:
    void sync_callback(const ImageMsg::ConstSharedPtr& img_l, const InfoMsg::ConstSharedPtr& info_l, 
                       const ImageMsg::ConstSharedPtr& img_r, const InfoMsg::ConstSharedPtr& info_r) {
        
        auto mod_img_l = std::const_pointer_cast<ImageMsg>(img_l);
        auto mod_info_l = std::const_pointer_cast<InfoMsg>(info_l);
        auto mod_img_r = std::const_pointer_cast<ImageMsg>(img_r);
        auto mod_info_r = std::const_pointer_cast<InfoMsg>(info_r);

        auto master_stamp = mod_img_l->header.stamp;

        if (has_last_stamp_) {
            rclcpp::Time current_time(master_stamp);
            double gap_sec = (current_time - last_stamp_).seconds();
            if (gap_sec > 0.08) {
                RCLCPP_WARN(this->get_logger(), 
                    "Frame drop detected in output stream! Gap: %.1f ms", gap_sec * 1000.0);
            }
        }
        last_stamp_ = rclcpp::Time(master_stamp);
        has_last_stamp_ = true;

        // Force synchronized timestamps across all 4 topics
        mod_info_l->header.stamp = master_stamp;
        mod_img_r->header.stamp = master_stamp;
        mod_info_r->header.stamp = master_stamp;

        // Override frame IDs to match SLAM parameters & TF tree
        mod_img_l->header.frame_id = "left_camera_optical_frame";
        mod_info_l->header.frame_id = "left_camera_optical_frame";
        mod_img_r->header.frame_id = "right_camera_optical_frame";
        mod_info_r->header.frame_id = "right_camera_optical_frame";
        
        // --- OVERRIDE ENTIRE MATRICES DIRECTLY FROM CALIBRATION YAMLS ---
        
        // LEFT CAMERA
        mod_info_l->width = 1280;
        mod_info_l->height = 720;
        mod_info_l->distortion_model = "plumb_bob";
        mod_info_l->d = {-0.023805, 0.024782, -0.002685, -0.002018, 0.000000};
        mod_info_l->k = {859.19722, 0.0, 654.42706, 
                         0.0, 642.77177, 355.56064, 
                         0.0, 0.0, 1.0};
        mod_info_l->r = {0.99970737, -0.01939622, -0.01445583, 
                         0.01942196, 0.99981003, 0.00164277, 
                         0.01442122, -0.00192305, 0.99989416};
        mod_info_l->p = {893.74551, 0.0, 657.54638, 0.0, 
                         0.0, 893.74551, 373.61695, 0.0, 
                         0.0, 0.0, 1.0, 0.0};

        // RIGHT CAMERA
        mod_info_r->width = 1280;
        mod_info_r->height = 720;
        mod_info_r->distortion_model = "plumb_bob";
        mod_info_r->d = {-0.049605, 0.058060, -0.001977, -0.003954, 0.000000};
        mod_info_r->k = {880.55194, 0.0, 643.20993, 
                         0.0, 660.19993, 397.00819, 
                         0.0, 0.0, 1.0};
        mod_info_r->r = {0.99977279, -0.01488711, -0.01525607, 
                         0.01485988, 0.99988779, -0.00189651, 
                         0.01528259, 0.00166937, 0.99988182};
        mod_info_r->p = {893.74551, 0.0, 657.54638, -53.188, 
                         0.0, 893.74551, 373.61695, 0.0, 
                         0.0, 0.0, 1.0, 0.0};
                         
        // -----------------------------------------------------------------

        pub_img_l_->publish(*mod_img_l);
        pub_info_l_->publish(*mod_info_l);
        pub_img_r_->publish(*mod_img_r);
        pub_info_r_->publish(*mod_info_r);

        synced_out_count_++;
    }

    void log_stats() {
        uint64_t l_in = left_in_count_.exchange(0);
        uint64_t r_in = right_in_count_.exchange(0);
        uint64_t synced = synced_out_count_.exchange(0);

        uint64_t drops_l = (l_in > synced) ? (l_in - synced) : 0;
        uint64_t drops_r = (r_in > synced) ? (r_in - synced) : 0;

        RCLCPP_INFO(this->get_logger(), 
            "[SYNC STATS] In (L/R): %lu / %lu | Synced Out: %lu | Dropped: L:%lu R:%lu", 
            l_in, r_in, synced, drops_l, drops_r);
    }

    message_filters::Subscriber<ImageMsg> sub_img_l_, sub_img_r_;
    message_filters::Subscriber<InfoMsg> sub_info_l_, sub_info_r_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    rclcpp::Publisher<ImageMsg>::SharedPtr pub_img_l_, pub_img_r_;
    rclcpp::Publisher<InfoMsg>::SharedPtr pub_info_l_, pub_info_r_;

    rclcpp::TimerBase::SharedPtr stats_timer_;

    std::atomic<uint64_t> left_in_count_;
    std::atomic<uint64_t> right_in_count_;
    std::atomic<uint64_t> synced_out_count_;

    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
    bool has_last_stamp_{false};
};

} // namespace stereo_sync_spoofer

// Register as a plugin library
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_sync_spoofer::StereoSyncSpoofer)
