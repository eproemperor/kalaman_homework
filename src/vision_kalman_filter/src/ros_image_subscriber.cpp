/**
 * @file ros_image_subscriber.cpp
 * @brief ROS2图像订阅器实现
 */

#include "vision_kalman_filter/ros_image_subscriber.h"

namespace vision_kalman_filter {

ROSImageSubscriber::ROSImageSubscriber(rclcpp::Node::SharedPtr node) 
    : node_(node),
      has_new_image_(false),
      image_timestamp_(0.0),
      frame_count_(0),
      current_fps_(0.0) {
    
    if (!node_) {
        throw std::runtime_error("ROSImageSubscriber: node pointer is null");
    }
    
    // 创建ImageTransport实例
    it_ = std::make_shared<image_transport::ImageTransport>(node_);
    
    // 订阅图像话题
    image_sub_ = std::make_shared<image_transport::Subscriber>(
        it_->subscribe("/image_raw", 10, 
                      std::bind(&ROSImageSubscriber::ImageCallback, this, std::placeholders::_1))
    );
    
    last_fps_time_ = node_->now();
    
    RCLCPP_INFO(node_->get_logger(), "图像订阅器已初始化，订阅话题: /image_raw");
}

ROSImageSubscriber::~ROSImageSubscriber() {
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "图像订阅器已关闭");
    }
}

void ROSImageSubscriber::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
        // 将ROS2图像转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = cv_ptr->image.clone();
            image_timestamp_.store(rclcpp::Time(msg->header.stamp).seconds());
            has_new_image_.store(true);
        }
        
        // 更新帧率
        UpdateFPS();
        
        // 通知等待的线程
        wait_cond_.notify_one();
        
    } catch (cv_bridge::Exception& e) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge异常: %s", e.what());
        }
    }
}

cv::Mat ROSImageSubscriber::GetLatestImage() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    return latest_image_.clone();
}

bool ROSImageSubscriber::HasNewImage() const {
    return has_new_image_.load();
}

double ROSImageSubscriber::GetImageTimestamp() const {
    return image_timestamp_.load();
}

double ROSImageSubscriber::GetFPS() const {
    return current_fps_.load();
}

bool ROSImageSubscriber::WaitForNextFrame(int timeout_ms) {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    has_new_image_.store(false);
    
    auto status = wait_cond_.wait_for(lock, std::chrono::milliseconds(timeout_ms));
    return status == std::cv_status::no_timeout;
}

void ROSImageSubscriber::ResetNewImageFlag() {
    has_new_image_.store(false);
}

void ROSImageSubscriber::UpdateFPS() {
    frame_count_++;
    rclcpp::Time now = node_->now();
    double dt = (now - last_fps_time_).seconds();
    
    if (dt >= 1.0) {
        current_fps_.store(frame_count_ / dt);
        frame_count_ = 0;
        last_fps_time_ = now;
        
        if (node_) {
            RCLCPP_DEBUG(node_->get_logger(), "当前FPS: %.2f", current_fps_.load());
        }
    }
}

}  // namespace vision_kalman_filter