#include "light_solver/solver_method.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>

namespace light_solver {

SolverMethod::SolverMethod() {}

// 计算像素差值（不含补偿，补偿将在外部应用）
Eigen::Vector2d SolverMethod::calculatePixelCenterDiff(
    const light_interfaces::msg::Armor& armor_msg,
    const Eigen::Vector2d& center_point) {
    Eigen::Vector2d diff;
    // 计算x差值（不含补偿）
    diff.x() = armor_msg.x - center_point.x();
    // 计算y差值（不含补偿）
    diff.y() = armor_msg.y - center_point.y();
    return diff;
}

// 滤波处理装甲板中心
Eigen::Vector2d SolverMethod::filterArmorCenter(
    const light_interfaces::msg::Armor& armor_msg,
    rclcpp::Time timestamp) {
    // 转换时间戳为秒（一欧元滤波器需要时间参数）
    double time = timestamp.seconds();
    // 分别对x和y坐标进行滤波，使用OneEuroFilter的filter()方法
    double filtered_x = x_filter_.filter(armor_msg.x, time);
    double filtered_y = y_filter_.filter(armor_msg.y, time);
    return Eigen::Vector2d(filtered_x, filtered_y);
}

// 设置滤波器参数，使用OneEuroFilter的单独参数设置方法
void SolverMethod::setFilterParams(double min_cutoff, double beta, double d_cutoff) {
    x_filter_.setMinCutoff(min_cutoff);
    x_filter_.setBeta(beta);
    x_filter_.setDCutoff(d_cutoff);
    
    y_filter_.setMinCutoff(min_cutoff);
    y_filter_.setBeta(beta);
    y_filter_.setDCutoff(d_cutoff);
}

// 设置追踪参数
void SolverMethod::setTrackingParams(
    double x_threshold,
    double y_threshold,
    double aim_duration_threshold,
    double max_target_ratio,
    double min_target_ratio,
    bool use_red_dot_detection,
    int red_dot_pixel_threshold,
    double aimed_reset_delay,
    double reset_duration) {
    x_threshold_ = x_threshold;
    y_threshold_ = y_threshold;
    aim_duration_threshold_ = aim_duration_threshold;
    max_target_ratio_ = max_target_ratio;
    min_target_ratio_ = min_target_ratio;
    aimed_reset_delay_ = aimed_reset_delay;
    reset_duration_ = reset_duration;
    
    // 新增：设置红点检测参数
    use_red_dot_detection_ = use_red_dot_detection;
    red_dot_pixel_threshold_ = red_dot_pixel_threshold;
}

// 新增：检测ROI内的红点（二值化图像中的白色像素）
bool SolverMethod::detectRedDotInROI(
    const cv::Mat& binary_image,
    const light_interfaces::msg::Armor& armor_msg) {
    // 检查图像是否为空
    if (binary_image.empty()) {
        return false;
    }
    
    // 计算装甲板边界框（确保在图像范围内）
    // 修复类型不匹配问题：将armor_msg的float类型转换为double
    int x1 = static_cast<int>(std::max(0.0, static_cast<double>(armor_msg.x - armor_msg.width / 2)));
    int y1 = static_cast<int>(std::max(0.0, static_cast<double>(armor_msg.y - armor_msg.height / 2)));
    int x2 = static_cast<int>(std::min(static_cast<double>(binary_image.cols - 1), 
                                      static_cast<double>(armor_msg.x + armor_msg.width / 2)));
    int y2 = static_cast<int>(std::min(static_cast<double>(binary_image.rows - 1), 
                                      static_cast<double>(armor_msg.y + armor_msg.height / 2)));
    
    // 确保ROI有效
    if (x1 >= x2 || y1 >= y2) {
        return false;
    }
    
    // 提取ROI区域
    cv::Mat roi = binary_image(cv::Rect(x1, y1, x2 - x1, y2 - y1));
    
    // 计算白色像素数量（红点在二值化图像中为白色）
    int white_pixel_count = cv::countNonZero(roi);
    
    // 如果白色像素数量超过阈值，则认为检测到红点
    return white_pixel_count >= red_dot_pixel_threshold_;
}

// 更新瞄准状态（使用已补偿的差值）
AimState SolverMethod::updateAimState(
    const Eigen::Vector2d& compensated_diff,  // 已补偿的差值
    const light_interfaces::msg::Armor& armor_msg,
    double image_area,
    bool red_dot_detected) {
    
    // 计算目标面积和占比
    double target_area = armor_msg.width * armor_msg.height;
    double target_ratio = target_area / image_area;
    
    // 判断是否需要后退（目标过大）
    need_back_ = (target_ratio > max_target_ratio_);
    
    // 如果目标过小，强制设为未瞄准状态
    if (target_ratio < min_target_ratio_) {
        current_state_ = AimState::NOT_AIMING;
        is_timing_ = false;
        return current_state_;
    }
    
    // 根据不同模式判断是否在瞄准范围内
    bool in_range = false;
    
    if (use_red_dot_detection_) {
        // 红点检测模式：红点在框内即认为在范围内
        in_range = red_dot_detected;
    } else {
        // 原始差值模式：使用已补偿的差值判断
        in_range = (std::abs(compensated_diff.x()) <= x_threshold_) && 
                  (std::abs(compensated_diff.y()) <= y_threshold_);
    }

    // 更新状态机
    if (current_state_ == AimState::NOT_AIMING) {
        if (in_range) {
            // 进入瞄准状态，开始计时
            current_state_ = AimState::AIMING;
            is_timing_ = true;
            start_aim_time_ = std::chrono::steady_clock::now();
        }
    } 
    else if (current_state_ == AimState::AIMING) {
        if (!in_range) {
            // 超出范围，回到未瞄准状态
            current_state_ = AimState::NOT_AIMING;
            is_timing_ = false;
        } else {
            // 计算已瞄准时间
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - start_aim_time_;
            if (elapsed.count() >= aim_duration_threshold_) {
                // 达到持续时间，进入已瞄准状态，并记录开始时间
                current_state_ = AimState::AIMED;
                aimed_start_time_ = now;
            }
        }
    }
    else if (current_state_ == AimState::AIMED) {
        // 检查AIMED状态是否持续足够长时间
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> aimed_elapsed = now - aimed_start_time_;
        
        if (aimed_elapsed.count() >= aimed_reset_delay_) {
            // AIMED状态持续时间已到，进入重置状态
            current_state_ = AimState::RESETTING;
            reset_start_time_ = now;
        }
        else if (!in_range) {
            // 未达到重置时间但已超出范围，回到未瞄准状态
            current_state_ = AimState::NOT_AIMING;
            is_timing_ = false;
        }
    }
    else if (current_state_ == AimState::RESETTING) {
        // 检查重置状态是否持续足够长时间
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> reset_elapsed = now - reset_start_time_;
        
        if (reset_elapsed.count() >= reset_duration_) {
            // 重置时间已到，回到未瞄准状态，重新开始追踪
            current_state_ = AimState::NOT_AIMING;
            is_timing_ = false;
        }
    }

    return current_state_;
}

} // namespace light_solver
