#ifndef LIGHT_SOLVER_SOLVER_NODE_HPP
#define LIGHT_SOLVER_SOLVER_NODE_HPP
// ros2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"
// 新增：图像转换所需头文件
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <mutex>  // 用于线程安全

// project
#include "light_interfaces/msg/armor.hpp"
#include "light_interfaces/msg/send_data.hpp"
#include "light_utils/heartbeat.hpp"
#include "light_interfaces/srv/get_aim_state.hpp"

#include "light_solver/solver_method.hpp"

namespace light_solver {

class SolverNode : public rclcpp::Node {
public:
    SolverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // 核心算法
    SolverMethod solver_method_;

    // 图像中心点坐标和面积
    Eigen::Vector2d image_center_;
    double image_area_;  // 图像总面积
    bool has_image_center_ = false;

    // 订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<light_interfaces::msg::Armor>::SharedPtr armor_sub_;
    // 新增：二值化图像订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr binary_image_sub_;

    // 发布器
    rclcpp::Publisher<light_interfaces::msg::SendData>::SharedPtr send_data_pub_;

    // 服务服务器
    rclcpp::Service<light_interfaces::srv::GetAimState>::SharedPtr aim_state_service_;
    
    // 服务回调函数
    void getAimStateCallback(
        const std::shared_ptr<light_interfaces::srv::GetAimState::Request> request,
        std::shared_ptr<light_interfaces::srv::GetAimState::Response> response);

    // 滤波后的装甲板中心
    Eigen::Vector2d filtered_armor_center_;
    bool has_filtered_center_ = false;
    
    // 当前瞄准状态
    AimState current_aim_state_ = AimState::NOT_AIMING;

    // 最后接收信息的时间
    rclcpp::Time last_armor_time_;
    rclcpp::Time last_binary_image_time_;  // 新增：最后接收二值化图像的时间
    
    // 超时时间（秒）
    const double ARMOR_TIMEOUT = 0.5;
    const double BINARY_IMAGE_TIMEOUT = 1.0;  // 新增：二值化图像超时时间

    // 开关参数缓存
    bool use_filter_;           // 滤波开关
    bool enable_debug_;         // 调试信息开关
    bool use_red_dot_detection_;  // 新增：红点检测开关
    int red_dot_pixel_threshold_;  // 新增：红点像素阈值
    
    // 图像尺寸参数
    int image_width_;
    int image_height_;

    // 目标大小参数
    double max_target_ratio_;   // 目标最大比例（占图像总面积）
    double min_target_ratio_;   // 目标最小比例（占图像总面积）

    // 横向和纵向补偿参数（像素值）
    double horizontal_compensation_;  // 横向补偿（左右方向）
    double vertical_compensation_;    // 纵向补偿（上下方向）
    
    // 新增：AIMED状态自动重置参数
    double aimed_reset_delay_;  // AIMED状态自动重置延迟（秒）
    double reset_duration_;     // 重置持续时间（秒）

    // 新增：存储最新的二值化图像和互斥锁
    cv::Mat latest_binary_image_;
    std::mutex binary_image_mutex_;

    // 回调函数
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void armorCallback(const light_interfaces::msg::Armor::SharedPtr msg);
    void binaryImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);  // 新增：二值化图像回调
    
    // 检查装甲板信息是否超时的定时器回调
    void checkArmorTimeout();

    // 参数回调
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters);

    // Heartbeat
    HeartBeatPublisher::SharedPtr heartbeat_pub_;
    
    // 定时器，用于检查装甲板信息是否超时
    rclcpp::TimerBase::SharedPtr armor_timeout_timer_;
};

} // namespace light_solver

#endif // LIGHT_SOLVER_SOLVER_NODE_HPP
