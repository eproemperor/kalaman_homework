#ifndef VISION_KALMAN_FILTER_KALMAN_FILTER_H
#define VISION_KALMAN_FILTER_KALMAN_FILTER_H

#include <cmath>
#include <opencv2/core.hpp>

//卡尔曼滤波器全局配置参数
namespace KalmanConfig {
    const double Q_POS = 0.4;      // 位置过程噪声
    const double Q_VEL = 0.6;      // 速度过程噪声
    const double Q_ACC = 5;      // 加速度过程噪声
    const double R_MEAS = 2.0;     // 观测噪声
    
    const float CANNON_CENTER_X = 575.5f;
    const float CANNON_CENTER_Y = 611.5f;
    const float CANNON_RADIUS = 22.9f;
    
    const float BULLET_SPEED = 600.0f;      // 子弹速度
}

class KalmanTracker {
private:
    double x_;   
    double y_;      
    double vx_;     
    double vy_;    
    double ax_;     
    double ay_;     
    
    // X方向：位置、速度、加速度的协方差矩阵 [Pxx, Pxv, Pxa; Pvx, Pvv, Pva; Pax, Pav, Paa]
    double p_xx_;  
    double p_xv_;   
    double p_xa_;  
    double p_vv_;  
    double p_va_;  
    double p_aa_;   
    
    double p_yy_;   
    double p_yv_;   
    double p_ya_;   
    double p_vv_y_; 
    double p_va_y_; 
    double p_aa_y_; 
    
    double q_pos_;  
    double q_vel_;  
    double q_acc_;  
    double r_meas_; 
    
    double last_time_;  
    double dt_;         
    double dt2_;        
    double dt3_;        
    
    //炮台
    const double cannon_x_;  
    const double cannon_y_;  
    
    cv::Point2f last_measured_pos_;  // 上一帧的观测位置
    double last_measured_time_;      // 上一帧观测时间
    bool has_last_measurement_;      
    
    double smoothed_ax_;  
    double smoothed_ay_;  
    int acc_samples_;     
    
public:
    int invalid_frames;

    KalmanTracker();
    
    double process(const cv::Point2f& measured_pos);
    
    cv::Point2f getPosition() const { 
        return cv::Point2f(x_, y_); 
    }
    
    cv::Point2f getVelocity() const { 
        return cv::Point2f(vx_, vy_); 
    }
    
    cv::Point2f getAcceleration() const { 
        return cv::Point2f(ax_, ay_); 
    }
    
    void reset(const cv::Point2f& init_pos);
    
private:
    
    inline void predict();
    
    inline void update(double zx, double zy);
};

#endif // VISION_KALMAN_FILTER_KALMAN_FILTER_H