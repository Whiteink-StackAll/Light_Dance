#ifndef LIGHT_SOLVER_SOLVER_METHOD_HPP
#define LIGHT_SOLVER_SOLVER_METHOD_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>  // 新增：OpenCV头文件

// ROS 2 消息类型
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose.hpp"

// 自定义消息类型
#include "light_interfaces/msg/armor.hpp"

// 一欧元滤波器
#include "light_solver/one_euro_filter.hpp"

namespace light_solver {

// 瞄准状态枚举
enum class AimState {
    NOT_AIMING,  // 没有瞄准
    AIMING,      // 正在瞄准
    AIMED,       // 已瞄准
    RESETTING    // 重置中（新增状态）
};

class SolverMethod {
public:
    SolverMethod();

    // 计算离中心点差值（不含补偿，补偿将在外部应用）
    Eigen::Vector2d calculatePixelCenterDiff(
        const light_interfaces::msg::Armor& armor_msg,
        const Eigen::Vector2d& center_point);

    // 应用一欧元滤波处理装甲板中心点
    Eigen::Vector2d filterArmorCenter(
        const light_interfaces::msg::Armor& armor_msg,
        rclcpp::Time timestamp);

    // 设置滤波器参数
    void setFilterParams(double min_cutoff, double beta, double d_cutoff);

    // 更新瞄准状态（核心追踪功能）
    AimState updateAimState(
        const Eigen::Vector2d& compensated_diff,  // 已补偿的差值
        const light_interfaces::msg::Armor& armor_msg,
        double image_area,
        bool red_dot_detected = false);  // 新增：红点检测结果

    // 设置追踪参数
    void setTrackingParams(
        double x_threshold,           // x轴阈值（像素）
        double y_threshold,           // y轴阈值（像素）
        double aim_duration_threshold,// 瞄准持续时间阈值（秒）
        double max_target_ratio,      // 目标最大比例（占图像总面积）
        double min_target_ratio,      // 目标最小比例（占图像总面积）
        bool use_red_dot_detection = false,  // 新增：红点检测开关
        int red_dot_pixel_threshold = 5,     // 新增：红点像素阈值
        double aimed_reset_delay = 5.0,      // 新增：AIMED状态自动重置延迟（秒）
        double reset_duration = 1.0          // 新增：重置持续时间（秒）
    );

    // 新增：检测ROI内的红点（二值化图像中的白色区域）
    bool detectRedDotInROI(
        const cv::Mat& binary_image,
        const light_interfaces::msg::Armor& armor_msg);

    // 获取是否需要后退的标志
    bool getNeedBack() const { return need_back_; }

private:
    // 滤波器
    light_solver::OneEuroFilter x_filter_;
    light_solver::OneEuroFilter y_filter_;

    // 追踪状态相关
    AimState current_state_ = AimState::NOT_AIMING;  // 当前状态
    bool is_timing_ = false;                          // 是否正在计时
    std::chrono::time_point<std::chrono::steady_clock> start_aim_time_;  // 开始瞄准时间
    std::chrono::time_point<std::chrono::steady_clock> aimed_start_time_;  // 开始AIMED状态的时间
    std::chrono::time_point<std::chrono::steady_clock> reset_start_time_;  // 开始重置的时间

    // 目标大小判断相关
    bool need_back_ = false;                          // 是否需要后退
    double max_target_ratio_ = 0.3;                   // 目标最大比例
    double min_target_ratio_ = 0.05;                  // 目标最小比例

    // 追踪参数（可调节）
    double x_threshold_ = 10.0;        // x轴阈值（像素）
    double y_threshold_ = 10.0;        // y轴阈值（像素）
    double aim_duration_threshold_ = 0.5;   // 瞄准持续时间阈值（秒）
    double aimed_reset_delay_ = 5.0;        // AIMED状态自动重置延迟（秒）
    double reset_duration_ = 1.0;           // 重置持续时间（秒）
    
    // 新增：红点检测参数
    bool use_red_dot_detection_ = false;  // 是否使用红点检测模式
    int red_dot_pixel_threshold_ = 5;     // 红点像素数量阈值
};

} // namespace light_solver

#endif // LIGHT_SOLVER_SOLVER_METHOD_HPP
