/**
 * 该模块提供装甲板识别所需的核心功能：
 * - 炮台检测与颜色识别
 * - 图像区域分割（炮台区域/装甲板区域）
 * - 敌方/己方颜色掩码提取
 * - 装甲板中心点定位
 * 
 */

#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>


enum ArmorColor {
    COLOR_RED = 0,      // 红色装甲板/炮台
    COLOR_BLUE = 1,     // 蓝色装甲板/炮台
    COLOR_UNKNOWN = -1  // 未知颜色
};

/**
 * @brief 炮台信息结构体
 * 
 * 存储检测到的炮台相关信息
 */
struct CannonInfo {
    ArmorColor color;           // 炮台颜色
    cv::Point2f center;         // 圆心坐标（图像坐标系）
    float radius;               // 圆半径（像素）
    
    CannonInfo() : color(COLOR_UNKNOWN), center(0, 0), radius(0) {}
    
    CannonInfo(ArmorColor c, cv::Point2f pt, float r) : color(c), center(pt), radius(r) {}
    /**
     * c 炮台颜色
     * pt 圆心坐标
     * r 半径
     */
};

/**
 * @brief OpenCV工具类，提供RoboMaster装甲板识别相关功能
 */
class openCV_tools {
private:
    cv::Mat current_frame;
    
    CannonInfo cannon_info;
    
    ArmorColor self_color;
    
    const cv::Scalar YELLOW_LOWER = cv::Scalar(20, 50, 50);
    const cv::Scalar YELLOW_UPPER = cv::Scalar(35, 255, 255);
    
    /** @brief 红色阈值（HSV色彩空间下红色分布在两个区间） */
    const cv::Scalar RED_LOWER1 = cv::Scalar(0, 100, 100);    // 红色区间1下限
    const cv::Scalar RED_UPPER1 = cv::Scalar(10, 255, 255);   // 红色区间1上限
    const cv::Scalar RED_LOWER2 = cv::Scalar(160, 100, 100);  // 红色区间2下限
    const cv::Scalar RED_UPPER2 = cv::Scalar(180, 255, 255);  // 红色区间2上限
    
    /** @brief 蓝色阈值 */
    const cv::Scalar BLUE_LOWER = cv::Scalar(100, 100, 100);
    const cv::Scalar BLUE_UPPER = cv::Scalar(140, 255, 255);
    
    const int MORPH_KERNEL_SIZE = 3;//形态学操作核(结构元素)大小
    
    const int MIN_ARMOR_AREA = 1500;//最小装甲板面积（像素）50*30像素
    
    const int MIN_COLOR_PIXELS = 50;//最小颜色像素数量（用于判断ROI颜色）
 
    const int CANNON_REGION_HEIGHT_RATIO = 5;    //炮台区域占图像高度的比例分母

    //const int 
    
    
    
    //用于提取图像中的黄色背景区域
    cv::Mat getYellowMask(const cv::Mat& frame) {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, YELLOW_LOWER, YELLOW_UPPER, mask);
        return mask;
    }
    
    //获取指定颜色的掩码
    cv::Mat getColorMask(const cv::Mat& frame, ArmorColor color) {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        //对于红色，由于HSV色彩空间的特性（分布在色相值的两端，不是一个连续的区间），需要合并两个区间
        if (color == COLOR_RED) {
            cv::Mat mask1, mask2;
            cv::inRange(hsv, RED_LOWER1, RED_UPPER1, mask1);
            cv::inRange(hsv, RED_LOWER2, RED_UPPER2, mask2);
            mask = mask1 | mask2;
        } else if (color == COLOR_BLUE) {
            cv::inRange(hsv, BLUE_LOWER, BLUE_UPPER, mask);
        } else {
            mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        }
        
        return mask;
    }
    
    /**
     * 应用形态学操作
     * 
     * 执行开运算和闭运算：
     *  开运算：先腐蚀后膨胀，去除噪声
     *  闭运算：先膨胀后腐蚀，填充孔洞
     */
    cv::Mat morphologyProcess(const cv::Mat& initial_img) {
        cv::Mat result;
        //kernel形态学操作中的探针
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                                                   cv::Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
        cv::morphologyEx(initial_img, result, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);
        return result;
    }
    
    // 获取炮台检测区域
    cv::Rect getCannonRegion() const {
        int height = current_frame.rows;
        int width = current_frame.cols;
        int cannon_region_height = height / CANNON_REGION_HEIGHT_RATIO;
        return cv::Rect(0, height - cannon_region_height, 
                        width, cannon_region_height);
    }
    
    //获取装甲板检测区域
    cv::Rect getArmorRegion() const {
        int height = current_frame.rows;
        int width = current_frame.cols;
        int cannon_region_height = height / CANNON_REGION_HEIGHT_RATIO;
        
        return cv::Rect(0, 0, width, height - cannon_region_height );
    }

public:
    
    openCV_tools() : self_color(COLOR_UNKNOWN) {}
    
    void setFrame(const cv::Mat& frame) {
        current_frame = frame.clone();
    }
    
    /**
     * @brief 识别炮台
     * @return 炮台信息结构体
     * 
     * 处理流程：
     * 1. 在炮台区域内进行霍夫圆检测
     * 2. 选择第一个检测到的圆作为炮台
     * 3. 在炮台周围提取ROI判断颜色
     * 4. 更新己方颜色和炮台信息
     */
    CannonInfo DetectCannon() {
        cv::Mat gray;
        cv::cvtColor(current_frame, gray, cv::COLOR_BGR2GRAY);
        
        cv::Rect cannon_region = getCannonRegion();
        cv::Mat cannon_area = gray(cannon_region);
        
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(cannon_area, circles, cv::HOUGH_GRADIENT, 1, 50, 100, 30, 20, 60);
        
        if (!circles.empty()) {
            cv::Vec3f circle = circles[0];
            // 将坐标映射回原图
            cv::Point2f center(circle[0], circle[1] + cannon_region.y);
            float radius = circle[2];
            
            // 在炮台周围提取ROI用于颜色判断
            cv::Rect roi_rect(
                std::max(0, (int)center.x - 20),
                std::max(0, (int)center.y - 20),
                std::min(40, current_frame.cols - (int)center.x + 20),
                std::min(40, current_frame.rows - (int)center.y + 20)
            );
            cv::Mat roi = current_frame(roi_rect);
            
            cv::Mat red_mask = getColorMask(roi, COLOR_RED);
            cv::Mat blue_mask = getColorMask(roi, COLOR_BLUE);
            
            int red_pixels = cv::countNonZero(red_mask);
            int blue_pixels = cv::countNonZero(blue_mask);
            
            ArmorColor color = (red_pixels > blue_pixels) ? COLOR_RED : COLOR_BLUE;
            cannon_info = CannonInfo(color, center, radius);
            self_color = color;
        }
        
        return cannon_info;
    }
    
    /**
     * @brief 获取非黄色区域
     * @return 二值图像，非黄色区域为白色
     * 
     * 用于提取可能包含装甲板的区域
     */
    cv::Mat GetNonYellowRegionsInArmorArea() {
        cv::Mat yellow_mask = getYellowMask(current_frame);
        cv::Mat non_yellow;
        cv::bitwise_not(yellow_mask, non_yellow);
        
        // 只保留装甲板区域的非黄色区域
        cv::Mat armor_region_mask = cv::Mat::zeros(current_frame.size(), CV_8UC1);
        cv::rectangle(armor_region_mask, getArmorRegion(), cv::Scalar(255), -1);
        
        cv::Mat result;
        cv::bitwise_and(non_yellow, armor_region_mask, result);
        result = morphologyProcess(result);
        
        return result;
    }
    
    /**
     * @brief 获取敌方颜色掩码（限制在装甲板区域）
     * @return 二值图像，敌方颜色区域为白色
     * 
     * 根据己方颜色确定敌方颜色
     */
    cv::Mat GetEnemyColorMaskInArmorArea() {
        ArmorColor enemy_color = (self_color == COLOR_RED) ? COLOR_BLUE : COLOR_RED;
        cv::Mat color_mask = getColorMask(current_frame, enemy_color);
        
        cv::Mat armor_region_mask = cv::Mat::zeros(current_frame.size(), CV_8UC1);
        cv::rectangle(armor_region_mask, getArmorRegion(), cv::Scalar(255), -1);
        
        cv::Mat result;
        cv::bitwise_and(color_mask, armor_region_mask, result);
        
        return result;
    }
    
    /**
     * @brief 获取己方颜色掩码（限制在装甲板区域）
     * @return 二值图像，己方颜色区域为白色
     */
    cv::Mat GetFriendlyColorMaskInArmorArea() {
        cv::Mat color_mask = getColorMask(current_frame, self_color);
        
        // 只保留装甲板区域的颜色
        cv::Mat armor_region_mask = cv::Mat::zeros(current_frame.size(), CV_8UC1);
        cv::rectangle(armor_region_mask, getArmorRegion(), cv::Scalar(255), -1);
        
        cv::Mat result;
        cv::bitwise_and(color_mask, armor_region_mask, result);
        
        return result;
    }
    
    /**
     * @brief 获取指定颜色掩码（对外接口，可指定区域）
     * @return 二值掩码图像
     */
    cv::Mat GetColorMask(ArmorColor color, bool limit_to_armor_area = true) {
        cv::Mat color_mask = getColorMask(current_frame, color);
        
        if (limit_to_armor_area) {
            cv::Mat armor_region_mask = cv::Mat::zeros(current_frame.size(), CV_8UC1);
            cv::rectangle(armor_region_mask, getArmorRegion(), cv::Scalar(255), -1);
            cv::bitwise_and(color_mask, armor_region_mask, color_mask);
        }
        
        return color_mask;
    }
    
    /**
     * @brief 寻找敌方装甲板中心点
     * @return 装甲板中心点坐标向量
     * 
     * 处理流程：
     * 1. 获取装甲板区域的非黄色区域
     * 2. 获取敌方颜色掩码
     * 3. 提取轮廓并筛选
     * 4. 检查轮廓内是否包含足够多的敌方颜色像素
     * 5. 计算符合条件的轮廓中心
     */
    std::vector<cv::Point2f> FindEnemyArmorCenters() {
        std::vector<cv::Point2f> armor_centers;
        
        cv::Mat non_yellow = GetNonYellowRegionsInArmorArea();
        cv::Mat enemy_mask = GetEnemyColorMaskInArmorArea();
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(non_yellow, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < MIN_ARMOR_AREA) continue;
            
            cv::Rect bbox = cv::boundingRect(contour);
            
            // 确保边界框在装甲板区域内
            if (bbox.y + bbox.height > getArmorRegion().y + getArmorRegion().height) {
                continue;  // 跳过延伸到炮台区域的轮廓
            }
            
            cv::Mat roi_enemy = enemy_mask(bbox);
            int enemy_pixels = cv::countNonZero(roi_enemy);
            
            if (enemy_pixels > MIN_COLOR_PIXELS) {
                cv::Moments m = cv::moments(contour);
                if (m.m00 != 0) {
                    cv::Point2f center(m.m10/m.m00, m.m01/m.m00);
                    armor_centers.push_back(center);
                }
            }
        }
        //for (auto& point : armor_centers) {
        //    point.y += 6.0f;  
        //}
        return armor_centers;
    }
    
    /**
     * @brief 寻找己方装甲板中心点
     * @return 装甲板中心点坐标向量
     * 
     */
    std::vector<cv::Point2f> FindFriendlyArmorCenters() {
        std::vector<cv::Point2f> armor_centers;
        
        cv::Mat non_yellow = GetNonYellowRegionsInArmorArea();
        cv::Mat friendly_mask = GetFriendlyColorMaskInArmorArea();
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(non_yellow, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < MIN_ARMOR_AREA) continue;
            
            cv::Rect bbox = cv::boundingRect(contour);
            
            // 确保边界框在装甲板区域内
            if (bbox.y + bbox.height > getArmorRegion().y + getArmorRegion().height) {
                continue;  // 跳过延伸到炮台区域的轮廓
            }
            
            cv::Mat roi_friendly = friendly_mask(bbox);
            int friendly_pixels = cv::countNonZero(roi_friendly);
            
            if (friendly_pixels > MIN_COLOR_PIXELS) {
                cv::Moments m = cv::moments(contour);
                if (m.m00 != 0) {
                    cv::Point2f center(m.m10/m.m00, m.m01/m.m00);
                    armor_centers.push_back(center);
                }
            }
        }
        
        return armor_centers;
    }
    
    /**
     * @brief 绘制检测结果
     * 
     * 绘制内容包括：
     * - 区域分割线（装甲板区域绿色，炮台区域黄色）
     * - 炮台圆圈和中心点
     * - 敌方装甲板（红色圆点）
     * - 己方装甲板（蓝色圆点）
     * - 文本标签
     */
    void drawDetections(cv::Mat& frame, 
                       const std::vector<cv::Point2f>& enemy_centers,
                       const std::vector<cv::Point2f>& friendly_centers) {
        // 绘制区域分割线
        cv::Rect armor_region = getArmorRegion();
        cv::Rect cannon_region = getCannonRegion();
        
        // 绘制装甲板区域边界
        cv::rectangle(frame, armor_region, cv::Scalar(0, 255, 0), 1);//绿色
        cv::putText(frame, "Armor Region", 
                   cv::Point(armor_region.x + 10, armor_region.y + 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        
        // 绘制炮台区域边界
        cv::rectangle(frame, cannon_region, cv::Scalar(0, 255, 255), 1);//黄色
        cv::putText(frame, "Cannon Region", 
                   cv::Point(cannon_region.x + 10, cannon_region.y + 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        // 绘制炮台
        if (cannon_info.radius > 0) {
            cv::Scalar cannon_color = (cannon_info.color == COLOR_RED) ? 
                                      cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            cv::circle(frame, cannon_info.center, cannon_info.radius, cannon_color, 2);
            cv::drawMarker(frame, cannon_info.center, cv::Scalar(255, 255, 255), 
                          cv::MARKER_CROSS, 10, 1);
        }
        
        // 绘制敌方装甲板（红色标记）
        for (const auto& center : enemy_centers) {  //c++11 的范围 for 循环，用于遍历容器中的元素
            if (armor_region.contains(cv::Point(center.x, center.y))) {
                cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);

                //cv::line(frame, cv::Point(0, center.y), cv::Point(frame.cols, center.y), cv::Scalar(0, 255, 0));
                
                cv::putText(frame, "ENEMY", cv::Point(center.x - 20, center.y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            }
        }

        // 绘制己方装甲板（蓝色标记）
        for (const auto& center : friendly_centers) {
            if (armor_region.contains(cv::Point(center.x, center.y))) {
                cv::circle(frame, center, 5, cv::Scalar(255, 0, 0), -1);
                cv::putText(frame, "FRIEND", cv::Point(center.x - 20, center.y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
            }
        }
    }
};
