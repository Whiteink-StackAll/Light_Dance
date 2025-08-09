#ifndef LIGHT_SERIAL_UART_NODE_HPP_
#define LIGHT_SERIAL_UART_NODE_HPP_

// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// project
#include "light_interfaces/msg/send_data.hpp"
#include "light_utils/heartbeat.hpp"  // 引入心跳头文件
#include "light_serial/uart_driver.hpp"
#include "light_serial/uart_protocol.hpp"

namespace light_serial
{

class UARTNode : public rclcpp::Node
{
public:
  explicit UARTNode(const rclcpp::NodeOptions & options);
  ~UARTNode();

private:
  void init_parameters();
  void init_subscriber();
  void init_publisher();
  void init_timer();
  void init_uart();
  void send_data_callback(const light_interfaces::msg::SendData::SharedPtr msg);
  void virtual_serial_timer_callback();  // 虚拟串口定时发送回调
  void serial_health_check_timer_callback();  // 串口健康检查定时器回调
  bool restart_serial();  // 重启串口功能
  
  // 辅助函数：格式化字节为十六进制字符串
  std::string format_bytes(const std::vector<uint8_t>& bytes);

  std::unique_ptr<UARTDriver> uart_driver_;
  rclcpp::Subscription<light_interfaces::msg::SendData>::SharedPtr send_sub_;
  rclcpp::TimerBase::SharedPtr virtual_serial_timer_;  // 虚拟串口定时器
  rclcpp::TimerBase::SharedPtr serial_health_check_timer_;  // 串口健康检查定时器

  // 参数
  std::string port_name_;
  int baudrate_;
  double timestamp_offset_;
  bool enable_data_print_;
  int serial_mode_;  // 0:关闭, 1:实际串口, 2:虚拟串口
  bool enable_send_data_print_;  // 发送数据打印开关
  double virtual_serial_frequency_;  // 虚拟串口发送频率
  float virtual_x_diff_;  // 虚拟X差值
  float virtual_y_diff_;  // 虚拟Y差值
  uint16_t virtual_mode_;  // 虚拟模式
  int virtual_need_back_;  // 虚拟后退标志
  double read_frequency_;
  int max_failure_count_;  // 最大失败次数阈值
  double health_check_interval_;  // 健康检查间隔(秒)
  int max_restart_attempts_;  // 最大重启尝试次数
  double restart_cooldown_;  // 重启冷却时间(秒)
  bool enable_auto_restart_;  // 是否启用自动重启功能
  int restart_delay_;  // 重启前延迟时间(毫秒)
  
  // 状态变量
  int consecutive_failure_count_;  // 连续失败计数
  int total_restart_attempts_;  // 总重启尝试次数
  bool is_healthy_;  // 串口是否健康
  rclcpp::Time last_successful_operation_time_;  // 上次成功操作时间
  rclcpp::Time last_restart_time_;  // 上次重启时间

  // Heartbeat - 使用提供的HeartBeatPublisher类
  HeartBeatPublisher::SharedPtr heartbeat_pub_;
};

}  // namespace light_serial

#endif  // LIGHT_SERIAL_UART_NODE_HPP_
