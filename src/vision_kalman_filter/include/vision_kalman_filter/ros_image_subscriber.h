#ifndef VISION_KALMAN_FILTER_ROS_IMAGE_SUBSCRIBER_H
#define VISION_KALMAN_FILTER_ROS_IMAGE_SUBSCRIBER_H

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>  // 用于压缩图像传输
#include <cv_bridge/cv_bridge.h>                 // ROS图像与OpenCV图像转换
#include <sensor_msgs/msg/image.hpp>             // ROS图像消息类型
#include <opencv2/opencv.hpp>                    
#include <memory>                                 // 智能指针
#include <mutex>                                  // 互斥锁
#include <atomic>                                 // 原子操作
#include <condition_variable>                      // 条件变量
#include <chrono>                                 // 时间相关

namespace vision_kalman_filter {

/**
 * @brief ROS2图像订阅器类
 * 
 * 提供以下功能：
 * - 订阅ROS2图像话题并自动转换为OpenCV格式
 * - 线程安全地获取最新图像帧
 * - 检测是否有新图像到达
 * - 统计接收帧率(FPS)
 * - 支持等待新帧到达的同步机制
 */
class ROSImageSubscriber {
public:
    explicit ROSImageSubscriber(rclcpp::Node::SharedPtr node);
    
    ~ROSImageSubscriber();
    
    cv::Mat GetLatestImage();
    
    bool HasNewImage() const;
    
    double GetImageTimestamp() const;
    
    double GetFPS() const;
    
    bool WaitForNextFrame(int timeout_ms = 1000);
    
    void ResetNewImageFlag();

private:
    void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    
    void UpdateFPS();

private:
    rclcpp::Node::SharedPtr node_;                    
    std::shared_ptr<image_transport::ImageTransport> it_;  // 图像传输对象，支持压缩传输
    std::shared_ptr<image_transport::Subscriber> image_sub_;  // 图像订阅者
    
    cv::Mat latest_image_;                            
    mutable std::mutex image_mutex_;                   // 保护图像数据的互斥锁
    std::atomic<bool> has_new_image_;    
    std::atomic<double> image_timestamp_;
    
    rclcpp::Time last_fps_time_; 
    int frame_count_;                                   // 统计周期内的帧计数
    std::atomic<double> current_fps_;
    
    std::mutex wait_mutex_;                            // 条件变量使用的互斥锁
    std::condition_variable wait_cond_;                 // 用于等待新帧的条件变量
};

}

#endif  // VISION_KALMAN_FILTER_ROS_IMAGE_SUBSCRIBER_H