/**
 * @file main.cpp
 * @brief 视觉卡尔曼滤波器 - RM装甲板自瞄系统
 * 
 * 功能概述：
 * 1. 使用卡尔曼滤波器跟踪多个装甲板目标
 * 2. 实现多目标选择与优先级评估
 * 3. 防友伤机制：检测弹道上是否有友军
 * 4. 目标置信度评估与遮挡检测
 * 5. 串口通信控制炮台和发射
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <iomanip>
#include <iostream>
#include <memory>
#include <ctime>
#include <opencv2/highgui.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <map>
#include <deque>

#include "vision_kalman_filter/ros_image_subscriber.h"
#include "vision_kalman_filter/openCV_tools.h"
#include "vision_kalman_filter/kalman_filter.h"
#include "vision_kalman_filter/serial_comm.h"
#include "vision_kalman_filter/msg/serialcom.hpp"

using namespace vision_kalman_filter;
using namespace KalmanConfig;

namespace Color {    
    const std::string RESET = "\033[0m";      
    const std::string RED = "\033[31m";       
    const std::string GREEN = "\033[32m";    
    const std::string YELLOW = "\033[33m";    
    const std::string BLUE = "\033[34m";     
    const std::string MAGENTA = "\033[35m";  
    const std::string CYAN = "\033[36m";      
}

/**
 * @brief 装甲板信息结构体
 * 存储单个目标的完整状态信息
 */
struct TargetInfo {
    int id;                      
    cv::Point2f position;       
    cv::Point2f velocity;       
    int color;                   // 颜色：COLOR_RED, COLOR_BLUE, COLOR_UNKNOWN
    int hit_count;               
    bool is_active;              
    bool is_lost;                
    bool is_occluded;           
    bool is_enemy;               
    int age;                     // 存在帧数（从第一次检测到开始计数）
    float score;                 
    float confidence;            // 置信度（0-1，值越高越可靠）
    std::deque<float> recent_scores; // 双端队列，存储最近10帧的置信度
    
    TargetInfo() : id(-1), position(0, 0), velocity(0, 0), color(COLOR_UNKNOWN),
                   hit_count(0), is_active(true), is_lost(false), is_occluded(false),
                   is_enemy(false), age(0), score(0.0f), confidence(1.0f) {}
    
    /**
     * @brief 更新置信度（使用滑动平均）
     * @param new_score 当前帧计算的置信度
     */
    void updateConfidence(float new_score) {
        recent_scores.push_back(new_score);
        if (recent_scores.size() > 10) {  // 只保留最近10帧
            recent_scores.pop_front();
        }
        
        // 计算平均置信度，使置信度平滑变动，避免跳变
        float sum = 0;
        for (float s : recent_scores) {
            sum += s;
        }
        confidence = sum / recent_scores.size();
    }
    
    bool isReliable() const {
        return confidence > 0.6f && !is_occluded;  // 置信度>0.6且未被遮挡
    }    
    
    cv::Point2f getPredictedPosition(float time_sec = 0.2f) const {
        return position + velocity * time_sec;  
    }
};

/**
 * @brief 安全检测结果结构体
 * 存储安全检查的详细结果
 */
struct SafetyCheckResult {
    bool is_safe_to_fire;      
    bool has_friendly_in_path;  
    bool target_occluded;       
    bool target_confident;      
    float risk_level;           
    std::string warning_message; 
    
    SafetyCheckResult() : is_safe_to_fire(true), has_friendly_in_path(false),
                          target_occluded(false), target_confident(true),
                          risk_level(0.0f), warning_message("") {}
};

/**
 * @brief 主视觉节点类
 */
class VisionNode : public rclcpp::Node {
private:
    std::shared_ptr<ROSImageSubscriber> image_sub_;  
    //rclcpp::Publisher<vision_kalman_filter::msg::Serialcom>::SharedPtr command_pub;
    
    openCV_tools cv_tools_;      
    SerialComm serial_comm_;     
    bool serial_comm_ok_{false}; 
    
    std::vector<std::shared_ptr<KalmanTracker>> trackers_;     
    std::map<int, TargetInfo> target_info_;      // 目标信息映射表（ID -> 目标信息）
    std::map<int, int> tracker_lost_frames_;      
    std::map<int, bool> tracker_destroyed_;       
    
    std::vector<int> enemy_target_ids_;    
    std::vector<int> friendly_target_ids_;  
    
    int current_target_id_{-1};     
    int target_lock_counter_{0};    
    const int LOCK_FRAMES_REQUIRED = 5;   
    const int MAX_LOST_FRAMES = 30;         // 最大丢失帧数（超过则丢弃目标）
    
    int self_color_{COLOR_UNKNOWN};     
    int enemy_color_{COLOR_UNKNOWN};     
    CannonInfo cannon_info_;             
    bool cannon_detected_{false};        
    
    rclcpp::Time last_fire_time_;        
    rclcpp::Time last_output_time_;     
    rclcpp::Time last_safety_check_time_; 
    
    int frame_count_{0};        
    
    const int HITS_TO_KILL = 3;                     
    const int FIRE_INTERVAL_MS = 250;                // 发射间隔250ms（限制射速）
    const float MIN_FIRE_DISTANCE = 1.0f;            // 最小开火距离（像素）
    const float MAX_FIRE_DISTANCE = 650.0f;          // 最大开火距离（像素）
    const float MAX_ANGLE_ERROR = 5.0f;             
    
    const float FRIENDLY_RADIUS = 90.0f;               // 友军判定半径（像素）
    const float SAFE_ANGLE_MARGIN = 10.0f;             
    const float OCCLUSION_THRESHOLD = 30.0f;           // 遮挡阈值（像素，距离小于此值视为遮挡）
    const float MIN_TARGET_CONFIDENCE = 0.6f;          // 最小目标置信度
    const int MAX_OCCLUSION_FRAMES = 5;                // 最大允许遮挡帧数
    const float FRIENDLY_PREDICTION_TIME = 0.2f;       // 友军预测时间（秒）
    
    float smoothed_angle_{0.0f};              
    const float SMOOTH_FACTOR = 0.3f;            // 平滑因子（0-1，越小越平滑）
    
    SafetyCheckResult last_safety_check_;      
    
    std::atomic<bool> running_{true};           // 线程运行标志（原子操作，线程安全）
    
public:
    VisionNode() : Node("vision_node") {
        image_sub_ = std::make_shared<ROSImageSubscriber>(
            std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){})  
        );
        //command_pub = this->create_publisher<vision_kalman_filter::msg::Serialcom>(
        //    "control_command",
        //    10
        //);
        
        serial_comm_ok_ = serial_comm_.open();
        if (!serial_comm_ok_) {
            RCLCPP_WARN(this->get_logger(), "串口打开失败，请检查权限");
        } else {
            RCLCPP_INFO(this->get_logger(), "串口打开成功");
        }
        
        last_fire_time_ = this->now();
        last_output_time_ = this->now();
        last_safety_check_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "========== RM装甲板自瞄系统(V5.2.0) ==========");
        RCLCPP_INFO(this->get_logger(), "子弹速度: %.0f 像素/秒", BULLET_SPEED);
        RCLCPP_INFO(this->get_logger(), "目标需要击中次数: %d", HITS_TO_KILL);
        RCLCPP_INFO(this->get_logger(), "发射间隔: %d ms", FIRE_INTERVAL_MS);
        RCLCPP_INFO(this->get_logger(), "友军判定半径: %.1f 像素", FRIENDLY_RADIUS);
        RCLCPP_INFO(this->get_logger(), "友军预测时间: %.1f 秒", FRIENDLY_PREDICTION_TIME);
        RCLCPP_INFO(this->get_logger(), "====================================================");
        
        cv::namedWindow("装甲板跟踪", cv::WINDOW_NORMAL);
        cv::resizeWindow("装甲板跟踪", 1152, 648);
    }
    
    
    ~VisionNode() {
        running_ = false; 
        cv::destroyAllWindows();  
    }
    
private:
    void processImage() {
        if (!running_ || !image_sub_->HasNewImage()) return;
        
        cv::Mat frame = image_sub_->GetLatestImage();
        if (frame.empty()) return;  
        
        cv_tools_.setFrame(frame);
        frame_count_++;  
        
        if (!cannon_detected_) {
            cannon_info_ = cv_tools_.DetectCannon();
            if (cannon_info_.radius > 0) {  
                cannon_detected_ = true;
                self_color_ = cannon_info_.color;
                enemy_color_ = (self_color_ == COLOR_RED) ? COLOR_BLUE : COLOR_RED;
                RCLCPP_INFO(this->get_logger(), "检测到炮台 - 颜色: %s",
                           self_color_ == COLOR_RED ? "红色" : "蓝色");
            }
        }
        
        if (cannon_detected_) {
            std::vector<cv::Point2f> enemy_centers = cv_tools_.FindEnemyArmorCenters();
            
            std::vector<cv::Point2f> friendly_centers;
            if (self_color_ == COLOR_RED) {
                friendly_centers = cv_tools_.FindFriendlyArmorCenters();  
            } else if (self_color_ == COLOR_BLUE) {
                friendly_centers = cv_tools_.FindFriendlyArmorCenters();  
            }
            
            // 合并所有检测到的目标，并标记敌我属性
            std::vector<std::pair<cv::Point2f, bool>> all_detections;
            for (const auto& pos : enemy_centers) {
                all_detections.push_back({pos, true});  
            }
            for (const auto& pos : friendly_centers) {
                all_detections.push_back({pos, false}); 
            }
            
            updateTrackers(all_detections);      
            updateTargetClassification();        
            checkLostTargets();                  
            checkOcclusion();                    
            selectTarget();                       
            
            if (current_target_id_ >= 0) {
                last_safety_check_ = performSafetyCheck();  
            }
            
            drawResults(frame);  
            
            cv::imshow("装甲板跟踪", frame);
            cv::waitKey(1);  
            
            auto now = this->now();
            if (now - last_output_time_ >= std::chrono::seconds(1)) {
                outputStatus();  
                last_output_time_ = now;
            }
            
            if (current_target_id_ >= 0 && serial_comm_ok_) {
                fireAtTargetWithSafety();  
            }
        }
    }
    
    /**
     * @brief 统一更新所有跟踪器
     * @param detections 检测到的目标列表（位置 + 敌我标志）
     * 
     * 使用最近邻匹配算法，将检测到的目标与现有跟踪器关联
     */
    void updateTrackers(const std::vector<std::pair<cv::Point2f, bool>>& detections) {
        const double MATCH_THRESHOLD = 50.0;  
        
        std::vector<bool> matched_trackers(trackers_.size(), false);
        std::vector<bool> matched_detections(detections.size(), false);
        
        for (size_t i = 0; i < trackers_.size(); i++) {
            if (!trackers_[i] || tracker_destroyed_[i]) continue;
            cv::Point2f pred_pos = trackers_[i]->getPosition();
            
            double min_dist = MATCH_THRESHOLD;
            int best_match = -1;
            
            for (size_t j = 0; j < detections.size(); j++) {
                if (matched_detections[j]) continue;  
                
                double dist = cv::norm(pred_pos - detections[j].first);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_match = j;
                }
            }
            
            if (best_match >= 0) {
                trackers_[i]->process(detections[best_match].first);
                target_info_[i].position = detections[best_match].first;
                target_info_[i].velocity = trackers_[i]->getVelocity();
                target_info_[i].is_enemy = detections[best_match].second;
                target_info_[i].color = detections[best_match].second ? enemy_color_ : self_color_;
                target_info_[i].age++;  
                target_info_[i].is_lost = false;
                
                
                float confidence = calculateConfidence(detections[best_match].first, i);
                target_info_[i].updateConfidence(confidence);
                
                tracker_lost_frames_[i] = 0;  
                matched_trackers[i] = true;
                matched_detections[best_match] = true;
            } else {
                tracker_lost_frames_[i]++;
                target_info_[i].is_lost = true;
                
                // 如果丢失太久，降低置信度
                if (tracker_lost_frames_[i] > 5) {
                    target_info_[i].updateConfidence(0.3f);
                }
            }
        }
        
        for (size_t j = 0; j < detections.size(); j++) {
            if (!matched_detections[j]) {  
                auto tracker = std::make_shared<KalmanTracker>();
                tracker->reset(detections[j].first);  
                trackers_.push_back(tracker);
                
                int new_id = trackers_.size() - 1;  
                
                TargetInfo info;
                info.id = new_id;
                info.position = detections[j].first;
                info.is_enemy = detections[j].second;
                info.color = detections[j].second ? enemy_color_ : self_color_;
                info.age = 1;
                info.is_active = true;
                info.is_lost = false;
                info.is_occluded = false;
                info.hit_count = 0;
                
                float confidence = calculateConfidence(detections[j].first, new_id);
                info.updateConfidence(confidence);
                
                target_info_[new_id] = info;
                tracker_lost_frames_[new_id] = 0;
                tracker_destroyed_[new_id] = false;
            }
        }
    }
    
    void updateTargetClassification() {
        enemy_target_ids_.clear();
        friendly_target_ids_.clear();
        
        for (const auto& kv : target_info_) {
            int id = kv.first;
            const TargetInfo& info = kv.second;
            
            if (tracker_destroyed_[id]) continue;  
            
            if (info.is_enemy) {
                enemy_target_ids_.push_back(id);
            } else {
                friendly_target_ids_.push_back(id);
            }
        }
    }
    
    std::vector<cv::Point2f> getPredictedFriendlyPositions() {
        std::vector<cv::Point2f> predicted_positions;
        
        for (int id : friendly_target_ids_) {
            if (!trackers_[id] || tracker_destroyed_[id]) continue;
            
            cv::Point2f pred_pos = target_info_[id].getPredictedPosition(FRIENDLY_PREDICTION_TIME);
            predicted_positions.push_back(pred_pos);
        }
        
        return predicted_positions;
    }
    
    float calculateConfidence(const cv::Point2f& position, int tracker_id) {
        float confidence = 1.0f;
        
        float dx = position.x - CANNON_CENTER_X;
        float dy = position.y - CANNON_CENTER_Y;
        float distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance > MAX_FIRE_DISTANCE * 0.8f) {
            confidence *= 0.8f;  
        }
        
        if (tracker_lost_frames_[tracker_id] > 0) {
            confidence *= (1.0f - tracker_lost_frames_[tracker_id] * 0.1f);
        }
        
        if (target_info_[tracker_id].age < 5) {
            confidence *= (0.5f + target_info_[tracker_id].age * 0.1f);
        }
        
        return std::max(0.1f, std::min(1.0f, confidence));  
    }
    
    void checkOcclusion() {
        std::vector<cv::Point2f> friendly_positions = getPredictedFriendlyPositions();
        
        for (auto& kv : target_info_) {
            int id = kv.first;
            TargetInfo& target = kv.second;
            
            if (tracker_destroyed_[id]) continue;
            if (!target.is_enemy) continue;  // 只检查敌方目标是否被友方遮挡
            
            bool occluded = false;
            
            for (const auto& friendly : friendly_positions) {
                float dist_to_friendly = cv::norm(target.position - friendly);
                
                if (dist_to_friendly < OCCLUSION_THRESHOLD) {
                    occluded = true;
                    break;
                }
            }
            
            if (target.position.x < 50 || target.position.x > 1100 ||
                target.position.y < 50 || target.position.y > 600) {
                occluded = true;
            }
            
            target.is_occluded = occluded;
            
            if (occluded) {
                target.updateConfidence(target.confidence * 0.7f);
            }
        }
    }
    
    /**
     * @brief 检查弹道上是否有友军
     * @param target_pos 目标位置
     * @return true=弹道上有友军，false=安全
     * 
     * 几何计算：计算友军到弹道线的垂直距离
     */
    bool checkFriendlyInFirePath(const cv::Point2f& target_pos) {
        cv::Point2f direction = target_pos - cv::Point2f(CANNON_CENTER_X, CANNON_CENTER_Y);
        float path_length = cv::norm(direction);
        if (path_length < 1.0f) return false;
        
        direction = direction / path_length;  
        
        // 获取友军预测位置
        std::vector<cv::Point2f> friendly_positions = getPredictedFriendlyPositions();
        
        for (const auto& friendly_pos : friendly_positions) {
            cv::Point2f friendly_rel = friendly_pos - cv::Point2f(CANNON_CENTER_X, CANNON_CENTER_Y);
            
            // 计算友军在弹道方向上的投影距离
            float projection = friendly_rel.x * direction.x + friendly_rel.y * direction.y;
            
            // 如果友军在弹道方向上（投影为正）且在目标之前（投影小于路径长度）
            if (projection > 0 && projection < path_length) {
                // 计算垂直距离（友军到弹道线的距离）
                cv::Point2f projection_point = cv::Point2f(CANNON_CENTER_X, CANNON_CENTER_Y) + direction * projection;
                float vertical_dist = cv::norm(friendly_pos - projection_point);
                
                if (vertical_dist < FRIENDLY_RADIUS) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    /**
     * @brief 执行安全检查
     * @return 安全检查结果
     */
    SafetyCheckResult performSafetyCheck() {
        SafetyCheckResult result;
        
        if (current_target_id_ < 0) {
            result.is_safe_to_fire = false;
            result.warning_message = "无目标";
            return result;
        }
        
        TargetInfo& target = target_info_[current_target_id_];
        
        result.target_confident = target.isReliable();
        if (!result.target_confident) {
            result.risk_level += 0.3f;
        }
        
        result.target_occluded = target.is_occluded;
        if (target.is_occluded) {
            result.risk_level += 0.4f;
        }
        
        result.has_friendly_in_path = checkFriendlyInFirePath(target.position);
        if (result.has_friendly_in_path) {
            result.risk_level += 0.5f;
        }
        
        result.is_safe_to_fire = result.risk_level < 0.6f 
                               && result.target_confident 
                               && !result.target_occluded
                               && !result.has_friendly_in_path;
        
        if (!result.is_safe_to_fire) {
            if (result.has_friendly_in_path) {
                result.warning_message = "警告：弹道上有友军！";
            } else if (result.target_occluded) {
                result.warning_message = "目标被遮挡，停火等待";
            } else if (!result.target_confident) {
                result.warning_message = "目标置信度低，继续确认";
            } else {
                result.warning_message = "安全检测未通过";
            }
        }
        
        return result;
    }
    
    /**
     * @brief 检查丢失目标
     * 处理长时间未检测到的目标
     */
    void checkLostTargets() {
        for (auto& kv : tracker_lost_frames_) {
            int id = kv.first;
            int lost_frames = kv.second;
            
            if (lost_frames > MAX_LOST_FRAMES && !tracker_destroyed_[id]) {
                if (target_info_[id].is_enemy) {
                }
                
                tracker_destroyed_[id] = true;
                target_info_[id].is_active = false;
                
                if (id == current_target_id_) {
                    current_target_id_ = -1;
                    target_lock_counter_ = 0;
                }
            }
        }
    }
    
    /**
     * @brief 选择最优目标
     * 基于多因素评分选择当前要攻击的目标
     */
    void selectTarget() {
        if (current_target_id_ >= 0 && 
            current_target_id_ < static_cast<int>(trackers_.size()) && 
            trackers_[current_target_id_] && 
            !tracker_destroyed_[current_target_id_] &&
            tracker_lost_frames_[current_target_id_] < 10 &&
            target_info_[current_target_id_].isReliable() &&
            target_info_[current_target_id_].is_enemy) {  
            
            target_lock_counter_++;  
            return;  
        }
        current_target_id_ = -1;
        target_lock_counter_ = 0;
        
        std::vector<std::pair<int, float>> candidates; 
        
        for (int id : enemy_target_ids_) {
            if (tracker_destroyed_[id]) continue;
            if (tracker_lost_frames_[id] > 20) continue;
            if (!target_info_[id].isReliable()) continue;
            
            cv::Point2f pos = target_info_[id].position;
            
            float dx = pos.x - CANNON_CENTER_X;
            float dy = pos.y - CANNON_CENTER_Y;
            float distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < MIN_FIRE_DISTANCE || distance > MAX_FIRE_DISTANCE) continue;
            
            float score = 0.0f;
            
            score += 1000.0f / distance;
            
            float angle_to_target = std::atan2(dx, dy) * 180.0f / M_PI;
            float angle_diff = std::abs(angle_to_target - smoothed_angle_);
            if (angle_diff > 180) angle_diff = 360 - angle_diff;
            score += 10.0f / (angle_diff + 1.0f);             
            score *= (1.0f + target_info_[id].age / 100.0f);           
            int remaining = HITS_TO_KILL - target_info_[id].hit_count;
            score *= (1.0f + remaining * 0.3f);            
            float speed = cv::norm(target_info_[id].velocity);
            score *= (1.0f + speed / 500.0f);
            score *= target_info_[id].confidence;
            if (target_info_[id].is_occluded) {
                score *= 0.5f;
            }
            
            target_info_[id].score = score;
            candidates.push_back({id, score});
        }
        
        if (!candidates.empty()) {
            std::sort(candidates.begin(), candidates.end(),
                     [](const auto& a, const auto& b) { return a.second > b.second; });
            
            current_target_id_ = candidates[0].first;  
            target_lock_counter_ = 1; 
        }
    }
    
    /**
     * @brief 带安全检查的开火决策
     * 在开火前进行多重安全检查
     */
    void fireAtTargetWithSafety() {
        auto now = this->now();
        
        SafetyCheckResult safety = performSafetyCheck();
        
        if (!safety.is_safe_to_fire) {
            if (!safety.warning_message.empty()) {
                RCLCPP_WARN(this->get_logger(), "安全检测未通过: %s", 
                           safety.warning_message.c_str());
            }
            
            if (safety.target_occluded && 
                target_info_[current_target_id_].is_occluded &&
                tracker_lost_frames_[current_target_id_] > MAX_OCCLUSION_FRAMES) {
                
                RCLCPP_INFO(this->get_logger(), "目标被遮挡过久，切换目标");
                current_target_id_ = -1;
                target_lock_counter_ = 0;
            }
            
            return;
        }
        
        int elapsed_ms = (now - last_fire_time_).seconds() * 1000;
        if (elapsed_ms < FIRE_INTERVAL_MS) return;
        
        TargetInfo& target = target_info_[current_target_id_];
        
        float target_angle = trackers_[current_target_id_]->process(target.position);
        target_angle = -target_angle + 90.0f;  // 坐标系转换
        
        smoothed_angle_ = smoothed_angle_ * SMOOTH_FACTOR + 
                         target_angle * (1.0f - SMOOTH_FACTOR);
        
        serial_comm_.sendTurnCommand(static_cast<int>(smoothed_angle_));
        
        float angle_error = std::abs(smoothed_angle_ - target_angle);
        if (angle_error > 180) angle_error = 360 - angle_error;
        
        if (angle_error > MAX_ANGLE_ERROR) return;
        
        if (target_lock_counter_ < LOCK_FRAMES_REQUIRED) {
            target_lock_counter_++;
            return;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
        SafetyCheckResult quick_safety = performSafetyCheck();
        if (!quick_safety.is_safe_to_fire) {
            RCLCPP_WARN(this->get_logger(), "开火前安全检查未通过");
            return;
        }
        
        // 发送开火命令
        if (serial_comm_.sendFireCommand()) {
            last_fire_time_ = now;  
            
            target.hit_count++;
            
            if (target.hit_count >= HITS_TO_KILL) {
                tracker_destroyed_[current_target_id_] = true;
                target.is_active = false;
                
                current_target_id_ = -1;
                target_lock_counter_ = 0;
            }
        }
    }
    
    void drawResults(cv::Mat& frame) {
        std::vector<cv::Point2f> enemy_centers;
        std::vector<cv::Point2f> friendly_centers;
        
        for (int id : enemy_target_ids_) {
            if (!tracker_destroyed_[id]) {
                enemy_centers.push_back(target_info_[id].position);
            }
        }
        
        for (int id : friendly_target_ids_) {
            if (!tracker_destroyed_[id]) {
                friendly_centers.push_back(target_info_[id].position);
            }
        }
        
        cv_tools_.drawDetections(frame, enemy_centers, friendly_centers);
        
        std::vector<cv::Point2f> predicted_friendly = getPredictedFriendlyPositions();
        
        for (const auto& pred_pos : predicted_friendly) {
            cv::circle(frame, pred_pos, 15, cv::Scalar(255, 100, 100), 2, cv::LINE_AA);
            
            // 绘制速度向量
            for (int id : friendly_target_ids_) {
                if (cv::norm(target_info_[id].position - pred_pos) < 20) {
                    cv::Point2f vel_end = pred_pos + target_info_[id].velocity * 0.1f;
                    cv::arrowedLine(frame, pred_pos, vel_end, cv::Scalar(255, 150, 150), 1, cv::LINE_AA);
                    
                    cv::putText(frame, "PREDICTED", 
                               cv::Point(pred_pos.x - 40, pred_pos.y - 20),
                               cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 150, 150), 1);
                    break;
                }
            }
        }
        
        for (size_t i = 0; i < trackers_.size(); i++) {
            if (!trackers_[i] || tracker_destroyed_[i]) continue;
            
            cv::Point2f pos = target_info_[i].position;
            
            cv::Scalar color;
            if (tracker_destroyed_[i]) {
                color = cv::Scalar(128, 128, 128);  // 灰色 - 已摧毁
            } else if (target_info_[i].is_occluded) {
                color = cv::Scalar(255, 165, 0);    // 橙色 - 被遮挡
            } else if (static_cast<int>(i) == current_target_id_) {
                color = cv::Scalar(0, 255, 0);      // 绿色 - 当前目标
            } else if (target_info_[i].is_enemy) {
                color = cv::Scalar(0, 0, 255);      // 红色 - 敌方
            } else {
                color = cv::Scalar(255, 0, 0);      // 蓝色 - 友方
            }
            
            // 绘制目标框
            cv::circle(frame, pos, 12, color, 2);
            
            // 绘制速度向量
            cv::Point2f vel_end = pos + target_info_[i].velocity * 0.1f;
            cv::arrowedLine(frame, pos, vel_end, color, 2);
            
            // 标注信息
            char label[200];
            snprintf(label, sizeof(label), 
                    "%s ID:%zu %d/%d A:%d C:%.2f%s", 
                    target_info_[i].is_enemy ? "E" : "F",  // E=敌方，F=友方
                    i, target_info_[i].hit_count, HITS_TO_KILL,  // 击中次数/需要次数
                    target_info_[i].age,  
                    target_info_[i].confidence,  
                    target_info_[i].is_occluded ? " [O]" : "");  // [O]=遮挡
            cv::putText(frame, label, 
                       cv::Point(pos.x - 50, pos.y - 25),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            
            if (static_cast<int>(i) == current_target_id_) {
                cv::circle(frame, pos, 20, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
                
                // 绘制弹道线
                cv::line(frame, 
                        cv::Point(CANNON_CENTER_X, CANNON_CENTER_Y), 
                        cv::Point(pos.x, pos.y), 
                        cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            }
        }
        
        char info[400];
        snprintf(info, sizeof(info), 
                "CEL:%d | ENEMY:%zu | FRIEND:%zu | SERIAL:%s | SAFE:%s | FPS:%.1f",
                frame_count_,  
                enemy_target_ids_.size(),  
                friendly_target_ids_.size(), 
                serial_comm_ok_ ? "OK" : "ERR",  // 串口状态
                last_safety_check_.is_safe_to_fire ? "SAFE" : "UNSAFE",  
                image_sub_->GetFPS());  // 帧率
        cv::putText(frame, info, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        
        if (!last_safety_check_.warning_message.empty()) {
            cv::putText(frame, last_safety_check_.warning_message, 
                       cv::Point(10, 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }
        
        if (current_target_id_ >= 0) {
            char target_info[200];
            snprintf(target_info, sizeof(target_info),
                    "TARGET %d | LOCKING:%d | ANGLE:%.1f° | HIT:%d/%d | TRUSTED:%.2f | %s",
                    current_target_id_, target_lock_counter_, smoothed_angle_,
                    target_info_[current_target_id_].hit_count, HITS_TO_KILL,
                    target_info_[current_target_id_].confidence,
                    target_info_[current_target_id_].is_occluded ? "COVER" : "UNCOVER");
            cv::putText(frame, target_info, cv::Point(10, 90),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
    }
    
    void outputStatus() {
        RCLCPP_INFO(this->get_logger(), 
                   "帧:%d 敌方:%zu 友方:%zu 当前目标:%s 锁定:%d 安全:%s",
                   frame_count_, 
                   enemy_target_ids_.size(),
                   friendly_target_ids_.size(),
                   current_target_id_ >= 0 ? std::to_string(current_target_id_).c_str() : "无",
                   last_safety_check_.is_safe_to_fire ? "是" : "否");
        
        if (current_target_id_ >= 0) {
            TargetInfo& target = target_info_[current_target_id_];
            float dx = target.position.x - CANNON_CENTER_X;
            float dy = CANNON_CENTER_Y - target.position.y;
            float dist = std::sqrt(dx*dx + dy*dy);
            
            RCLCPP_INFO(this->get_logger(), 
                       "  目标%d - 距离:%.1f 速度:(%.1f,%.1f) 置信度:%.2f %s 击中:%d/%d",
                       current_target_id_, dist,
                       target.velocity.x, target.velocity.y,
                       target.confidence,
                       target.is_occluded ? "[遮挡]" : "",
                       target.hit_count, HITS_TO_KILL);
        }
    }
    
public:
    void run() {
        std::thread processing_thread([this]() {
            RCLCPP_INFO(this->get_logger(), "图像处理线程启动");
            while (running_ && rclcpp::ok()) {
                processImage();  
                std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
            }
        });
        
        processing_thread.detach(); 
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  
    
    auto node = std::make_shared<VisionNode>();  
    node->run();  
    
    rclcpp::spin(node);  
    rclcpp::shutdown(); 
    
    return 0;
}