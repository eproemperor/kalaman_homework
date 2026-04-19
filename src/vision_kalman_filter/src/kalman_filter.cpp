#include "vision_kalman_filter/kalman_filter.h"
#include <cmath>
#include <chrono>
#include <algorithm>

using namespace KalmanConfig;

KalmanTracker::KalmanTracker()
    : x_(CANNON_CENTER_X)
    , y_(CANNON_CENTER_Y)
    , vx_(0.0)
    , vy_(0.0)
    , ax_(0.0)
    , ay_(0.0)
    // X方向协方差初始化
    , p_xx_(1.0)
    , p_xv_(0.0)
    , p_xa_(0.0)
    , p_vv_(1.0)
    , p_va_(0.0)
    , p_aa_(1.0)
    // Y方向协方差初始化
    , p_yy_(1.0)
    , p_yv_(0.0)
    , p_ya_(0.0)
    , p_vv_y_(1.0)
    , p_va_y_(0.0)
    , p_aa_y_(1.0)
    , q_pos_(Q_POS)
    , q_vel_(Q_VEL)
    , q_acc_(Q_ACC)
    , r_meas_(R_MEAS)
    , last_time_(0.0)
    , dt_(0.01)
    , dt2_(0.0001)
    , dt3_(0.000001)
    , cannon_x_(CANNON_CENTER_X)
    , cannon_y_(CANNON_CENTER_Y)
    , last_measured_pos_(CANNON_CENTER_X, CANNON_CENTER_Y)
    , last_measured_time_(0.0)
    , has_last_measurement_(false)
    , smoothed_ax_(0.0)
    , smoothed_ay_(0.0)
    , acc_samples_(0)
    , invalid_frames(0)
{
    last_time_ = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    last_measured_time_ = last_time_;
}

double KalmanTracker::process(const cv::Point2f& measured_pos) {
    double current_time = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    
    if (last_time_ > 0) {
        dt_ = current_time - last_time_;
        if (dt_ > 0.1) dt_ = 0.01;
        if (dt_ < 0.001) dt_ = 0.01;
        
        dt2_ = dt_ * dt_;
        dt3_ = dt2_ * dt_;
    } else {
        dt_ = 0.01;
        dt2_ = 0.0001;
        dt3_ = 0.000001;
    }
    last_time_ = current_time;
    
    // 处理丢失数据
    if (measured_pos.x < 0 || measured_pos.y < 0) {
        invalid_frames++;
        if (invalid_frames > 5) {
            reset(cv::Point2f(cannon_x_, cannon_y_));
        }
        double pred_x = x_ + vx_ * dt_ + 0.5 * ax_ * dt2_;
        double pred_y = y_ + vy_ * dt_ + 0.5 * ay_ * dt2_;
        
        double dx = pred_x - cannon_x_;
        double dy = cannon_y_ - pred_y;
        double angle_rad = std::atan2(dx, dy);
        return angle_rad * 180.0 / M_PI;
    }
    invalid_frames = 0;
    
    predict();
    update(measured_pos.x, measured_pos.y);
    
    if (has_last_measurement_ && dt_ > 0) {
        // 计算瞬时速度
        double inst_vx = (measured_pos.x - last_measured_pos_.x) / dt_;
        double inst_vy = (measured_pos.y - last_measured_pos_.y) / dt_;
        
        // 计算瞬时加速度
        double dt_vel = current_time - last_measured_time_;
        if (dt_vel > 0.001 && dt_vel < 0.1) {
            double inst_ax = (inst_vx - vx_) / dt_vel;
            double inst_ay = (inst_vy - vy_) / dt_vel;
            
            smoothed_ax_ = smoothed_ax_ * (1 - alpha) + inst_ax * alpha;
            smoothed_ay_ = smoothed_ay_ * (1 - alpha) + inst_ay * alpha;
            
            smoothed_ax_ = std::max(-2000.0, std::min(2000.0, smoothed_ax_));
            smoothed_ay_ = std::max(-2000.0, std::min(2000.0, smoothed_ay_));
            
            // 如果差异较大，进行修正
            if (acc_samples_ > 10) {
                const double correction_strength = 0.3;
                ax_ = ax_ * (1 - correction_strength) + smoothed_ax_ * correction_strength;
                ay_ = ay_ * (1 - correction_strength) + smoothed_ay_ * correction_strength;
            }
            acc_samples_ = std::min(acc_samples_ + 1, 100);
        }
    }
    
    last_measured_pos_ = measured_pos;
    last_measured_time_ = current_time;
    has_last_measurement_ = true;
    
    double dx = x_ - cannon_x_;
    double dy = cannon_y_ - y_;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    double flight_time = distance / BULLET_SPEED;
    flight_time = std::min(flight_time, 0.3);
    flight_time = std::max(flight_time, 0.01);
    
    // s = s0 + v0*t + 0.5*a*t²
    double future_x = x_ + vx_ * flight_time + 0.5 * ax_ * flight_time * flight_time;
    double future_y = y_ + vy_ * flight_time + 0.5 * ay_ * flight_time * flight_time;
    
    if (y_ < 300) {
        double extra_factor = 1.0 + (cannon_y_ - y_) / 300.0;
        extra_factor = std::min(extra_factor, 2.0);
        
        // 如果加速度向上（负Y方向），增加补偿
        if (ay_ < 0) {
            extra_factor *= (1.0 + std::min(std::abs(ay_) / 500.0, 0.5));
        }
        future_x = x_ + vx_ * flight_time * extra_factor + 0.5 * ax_ * flight_time * flight_time;
    }
        
    const double COMPENSATION_FACTOR = 0.055;//高度距离补偿系数
    const double VEL_COMPENSATION_FACTOR = 0.917;//速度补偿系数
    const double ACC_COMPENSATION_FACTOR = 0.3;  // 加速度补偿系数
    const double HIGH_SPEED_THRESHOLD = 180.0;//高速阈值

    double compensation = 0.0;
    double abs_vx = std::abs(vx_);

    if (abs_vx > 0.01) {
        double base_comp = (cannon_y_ - y_) * COMPENSATION_FACTOR;
        if((cannon_y_ - y_) > 350){
            base_comp = base_comp + 20.0f;
        }
        double vel_comp = 0.0;
        if (abs_vx >= HIGH_SPEED_THRESHOLD) {
            double speed_factor = abs_vx / 100.0;
            vel_comp =  speed_factor * speed_factor * speed_factor * VEL_COMPENSATION_FACTOR;
        }
        double acc_comp = 0.0;
        if (std::abs(ax_) > 50.0) { 
            double acc_factor = std::min(std::abs(ax_) / 500.0, 0.5);
            if (ax_ * vx_ > 0) {
                acc_comp = acc_factor * ACC_COMPENSATION_FACTOR * 50.0;
            } else if (ax_ * vx_ < 0) {
                acc_comp = -acc_factor * ACC_COMPENSATION_FACTOR * 30.0;
            }
        }
        
        double total_comp = base_comp + vel_comp + acc_comp;
        compensation = (vx_ > 0) ? total_comp : -total_comp;
    }
    
    // 计算最终角度
    dx = future_x - cannon_x_ + compensation;
    dy = cannon_y_ - future_y;
    double angle_rad = std::atan2(dx, dy);
    double angle_deg = angle_rad * 180.0 / M_PI;
    
    if (angle_deg > 180) angle_deg -= 360;
    if (angle_deg < -180) angle_deg += 360;
    
    angle_deg = std::max(-90.0, std::min(90.0, angle_deg));
    
    return angle_deg;
}

void KalmanTracker::reset(const cv::Point2f& init_pos) {
    x_ = init_pos.x;
    y_ = init_pos.y;
    vx_ = 0.0;
    vy_ = 0.0;
    ax_ = 0.0;
    ay_ = 0.0;
    
    // 重置协方差
    p_xx_ = 1.0;
    p_xv_ = 0.0;
    p_xa_ = 0.0;
    p_vv_ = 1.0;
    p_va_ = 0.0;
    p_aa_ = 1.0;
    
    p_yy_ = 1.0;
    p_yv_ = 0.0;
    p_ya_ = 0.0;
    p_vv_y_ = 1.0;
    p_va_y_ = 0.0;
    p_aa_y_ = 1.0;
    
    smoothed_ax_ = 0.0;
    smoothed_ay_ = 0.0;
    acc_samples_ = 0;
    has_last_measurement_ = false;
    invalid_frames = 0;
    
    last_time_ = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    last_measured_time_ = last_time_;
}

void KalmanTracker::predict() {
    // 位置预测: x = x + vx*dt + 0.5*ax*dt²
    // 速度预测: vx = vx + ax*dt
    // 加速度预测: ax = ax (恒定加速度模型)
    
    double dt2_half = 0.5 * dt2_;
    
    // 状态预测
    double pred_x = x_ + vx_ * dt_ + ax_ * dt2_half;
    double pred_y = y_ + vy_ * dt_ + ay_ * dt2_half;
    double pred_vx = vx_ + ax_ * dt_;
    double pred_vy = vy_ + ay_ * dt_;
    double pred_ax = ax_;
    double pred_ay = ay_;
    
    // 状态转移矩阵 F = [1, dt, 0.5*dt²; 0, 1, dt; 0, 0, 1]
    double f11 = 1.0;
    double f12 = dt_;
    double f13 = dt2_half;
    double f22 = 1.0;
    double f23 = dt_;
    double f33 = 1.0;
    
    // 计算新的协方差: P' = F * P * F^T + Q
    // X方向协方差预测
    double new_p_xx = f11 * (f11 * p_xx_ + f12 * p_xv_ + f13 * p_xa_) +
                      f12 * (f11 * p_xv_ + f12 * p_vv_ + f13 * p_va_) +
                      f13 * (f11 * p_xa_ + f12 * p_va_ + f13 * p_aa_);
    
    double new_p_xv = f11 * (f22 * p_xv_ + f23 * p_xa_) +
                      f12 * (f22 * p_vv_ + f23 * p_va_) +
                      f13 * (f22 * p_va_ + f23 * p_aa_);
    
    double new_p_xa = f11 * (f33 * p_xa_) +
                      f12 * (f33 * p_va_) +
                      f13 * (f33 * p_aa_);
    
    double new_p_vv = f22 * (f22 * p_vv_ + f23 * p_va_) +
                      f23 * (f22 * p_va_ + f23 * p_aa_);
    
    double new_p_va = f22 * (f33 * p_va_) +
                      f23 * (f33 * p_aa_);
    
    double new_p_aa = f33 * (f33 * p_aa_);
    
    // 添加过程噪声 Q
    // Q = [q_pos, 0, 0; 0, q_vel, 0; 0, 0, q_acc]
    new_p_xx += q_pos_;
    new_p_vv += q_vel_;
    new_p_aa += q_acc_;
    
    // 更新X方向协方差
    p_xx_ = new_p_xx;
    p_xv_ = new_p_xv;
    p_xa_ = new_p_xa;
    p_vv_ = new_p_vv;
    p_va_ = new_p_va;
    p_aa_ = new_p_aa;
    
    // 更新状态
    x_ = pred_x;
    y_ = pred_y;
    vx_ = pred_vx;
    vy_ = pred_vy;
    ax_ = pred_ax;
    ay_ = pred_ay;
}

void KalmanTracker::update(double zx, double zy) {
    double s_x = p_xx_ + r_meas_;  
    double kx = p_xx_ / s_x;       
    double kvx = p_xv_ / s_x;     
    double kax = p_xa_ / s_x;     
    
    double innov_x = zx - x_;
    
    x_ = x_ + kx * innov_x;
    vx_ = vx_ + kvx * innov_x;
    ax_ = ax_ + kax * innov_x;
    
    double p_xx_new = p_xx_ - kx * p_xx_;
    double p_xv_new = p_xv_ - kx * p_xv_;
    double p_xa_new = p_xa_ - kx * p_xa_;
    double p_vv_new = p_vv_ - kvx * p_xv_;
    double p_va_new = p_va_ - kvx * p_xa_;
    double p_aa_new = p_aa_ - kax * p_xa_;
    
    p_xx_ = p_xx_new;
    p_xv_ = p_xv_new;
    p_xa_ = p_xa_new;
    p_vv_ = p_vv_new;
    p_va_ = p_va_new;
    p_aa_ = p_aa_new;
    
    double s_y = p_yy_ + r_meas_;
    double ky = p_yy_ / s_y;
    double kvy = p_yv_ / s_y;
    double kay = p_ya_ / s_y;
    
    double innov_y = zy - y_;
    
    y_ = y_ + ky * innov_y;
    vy_ = vy_ + kvy * innov_y;
    ay_ = ay_ + kay * innov_y;
    
    double p_yy_new = p_yy_ - ky * p_yy_;
    double p_yv_new = p_yv_ - ky * p_yv_;
    double p_ya_new = p_ya_ - ky * p_ya_;
    double p_vv_y_new = p_vv_y_ - kvy * p_yv_;
    double p_va_y_new = p_va_y_ - kvy * p_ya_;
    double p_aa_y_new = p_aa_y_ - kay * p_ya_;
    
    p_yy_ = p_yy_new;
    p_yv_ = p_yv_new;
    p_ya_ = p_ya_new;
    p_vv_y_ = p_vv_y_new;
    p_va_y_ = p_va_y_new;
    p_aa_y_ = p_aa_y_new;
}