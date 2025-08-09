#include <chrono>
#include <memory>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <thread>

#include "light_serial/uart_node.hpp" 
#include "light_utils/heartbeat.hpp"

namespace light_serial
{

UARTNode::UARTNode(const rclcpp::NodeOptions & options)
: Node("light_serial", options),
  port_name_("/dev/ttyACM0"),
  baudrate_(115200),
  timestamp_offset_(0.0),
  enable_data_print_(false),
  serial_mode_(0),
  enable_send_data_print_(true),
  virtual_serial_frequency_(10.0),
  virtual_x_diff_(0.0f),
  virtual_y_diff_(0.0f),
  virtual_mode_(0),
  virtual_need_back_(0),
  read_frequency_(100.0),
  max_failure_count_(3),
  health_check_interval_(1.0),
  max_restart_attempts_(5),
  restart_cooldown_(5.0),
  enable_auto_restart_(true),
  restart_delay_(1000),
  consecutive_failure_count_(0),
  total_restart_attempts_(0),
  is_healthy_(false),
  last_successful_operation_time_(this->now()),
  last_restart_time_(this->now() - rclcpp::Duration::from_seconds(restart_cooldown_ + 1.0))
{
  try {
    init_parameters();
    init_subscriber();
    init_publisher();
    init_timer();  // 初始化定时器，包括健康检查定时器
    
    // 初始化心跳
    heartbeat_pub_ = HeartBeatPublisher::create(this);
    
    // 根据模式初始化相应组件
    if (serial_mode_ == 1) {
      // 实际串口模式
      init_uart();
      RCLCPP_INFO(get_logger(), "已启用实际串口模式（仅发送）");
    } else if (serial_mode_ == 2) {
      // 虚拟串口模式
      init_uart();
      auto period = std::chrono::duration<double>(1.0 / virtual_serial_frequency_);
      virtual_serial_timer_ = create_wall_timer(
        period, std::bind(&UARTNode::virtual_serial_timer_callback, this));
      RCLCPP_INFO(get_logger(), "已启用虚拟串口模式，发送频率: %.1fHz", virtual_serial_frequency_);
    } else {
      // 关闭模式
      RCLCPP_INFO(get_logger(), "串口已关闭，不进行数据收发");
    }

    // 打印发送数据开关状态
    RCLCPP_INFO(get_logger(), "发送数据打印功能: %s", 
                enable_send_data_print_ ? "启用" : "禁用");

    // 打印重启参数
    RCLCPP_INFO(get_logger(), "串口自动重启参数: 功能%s, 最大失败次数=%d, 健康检查间隔=%.1fs, 最大重启次数=%d, 重启冷却=%.1fs, 重启延迟=%dms",
                enable_auto_restart_ ? "启用" : "禁用",
                max_failure_count_, health_check_interval_, max_restart_attempts_, restart_cooldown_, restart_delay_);

    RCLCPP_INFO(get_logger(), "UART节点初始化完成");
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "节点初始化失败: %s", e.what());
    rclcpp::shutdown();
  }
}

UARTNode::~UARTNode()
{
  if (uart_driver_) {
    uart_driver_->close();
  }
}

void UARTNode::init_parameters()
{
  // 模式选择参数
  serial_mode_ = this->declare_parameter("serial_mode", 0);
  
  // 硬件串口参数
  port_name_ = this->declare_parameter("port_name", "/dev/ttyACM0");
  baudrate_ = this->declare_parameter("baudrate", 115200);
  read_frequency_ = this->declare_parameter("read_frequency", 100.0);
  
  // 通用参数
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  enable_data_print_ = this->declare_parameter("enable_data_print", false);
  
  // 发送数据打印控制
  enable_send_data_print_ = this->declare_parameter("enable_send_data_print", true);
  
  // 虚拟串口参数
  virtual_serial_frequency_ = this->declare_parameter("virtual_serial_frequency", 10.0);
  virtual_x_diff_ = this->declare_parameter("virtual_x_diff", 0.0f);
  virtual_y_diff_ = this->declare_parameter("virtual_y_diff", 0.0f);
  virtual_mode_ = this->declare_parameter("virtual_mode", 0);
  virtual_need_back_ = this->declare_parameter("virtual_need_back", 0);
  
  // 串口健康检查和重启参数
  enable_auto_restart_ = this->declare_parameter("enable_auto_restart", true);
  max_failure_count_ = this->declare_parameter("max_failure_count", 3);
  health_check_interval_ = this->declare_parameter("health_check_interval", 1.0);
  max_restart_attempts_ = this->declare_parameter("max_restart_attempts", 5);
  restart_cooldown_ = this->declare_parameter("restart_cooldown", 5.0);
  restart_delay_ = this->declare_parameter("restart_delay", 1000);

  // 验证参数有效性
  if (baudrate_ != 9600 && baudrate_ != 115200) {
    RCLCPP_WARN(get_logger(), "波特率 %d 可能不被支持，将使用默认值 115200", baudrate_);
    baudrate_ = 115200;
  }

  if (virtual_serial_frequency_ <= 0) {
    RCLCPP_WARN(get_logger(), "无效的虚拟串口频率，将使用默认值 10.0Hz");
    virtual_serial_frequency_ = 10.0;
  }
  
  if (max_failure_count_ <= 0) {
    RCLCPP_WARN(get_logger(), "无效的最大失败次数，将使用默认值 3");
    max_failure_count_ = 3;
  }
  
  if (health_check_interval_ <= 0) {
    RCLCPP_WARN(get_logger(), "无效的健康检查间隔，将使用默认值 1.0秒");
    health_check_interval_ = 1.0;
  }
  
  if (max_restart_attempts_ < 0) {
    RCLCPP_WARN(get_logger(), "无效的最大重启次数，将使用默认值 5");
    max_restart_attempts_ = 5;
  }
  
  if (restart_cooldown_ < 0) {
    RCLCPP_WARN(get_logger(), "无效的重启冷却时间，将使用默认值 5.0秒");
    restart_cooldown_ = 5.0;
  }
  
  if (restart_delay_ < 0) {
    RCLCPP_WARN(get_logger(), "无效的重启延迟时间，将使用默认值 1000ms");
    restart_delay_ = 1000;
  }

  // 打印参数信息
  RCLCPP_INFO(get_logger(), "串口模式: %d (0=关闭, 1=实际串口, 2=虚拟串口)", serial_mode_);
  if (serial_mode_ == 1) {
    RCLCPP_INFO(get_logger(), "硬件串口参数: %s @ %d, 读取频率: %.1fHz",
                port_name_.c_str(), baudrate_, read_frequency_);
  } else if (serial_mode_ == 2) {
    RCLCPP_INFO(get_logger(), "虚拟串口参数: 频率: %.1fHz, X: %.2f, Y: %.2f, 模式: %d, 需要后退: %d",
                virtual_serial_frequency_,
                virtual_x_diff_,
                virtual_y_diff_,
                virtual_mode_,
                virtual_need_back_);
  }
}

void UARTNode::init_subscriber()
{
  // 实际串口模式订阅消息
  if (serial_mode_ == 1) {
    send_sub_ = create_subscription<light_interfaces::msg::SendData>(
      "send_data", 10, std::bind(&UARTNode::send_data_callback, this, std::placeholders::_1));
  }
}

void UARTNode::init_publisher()
{
  // 当前不启用接收发布者
}

void UARTNode::init_timer()
{
  // 初始化串口健康检查定时器
  auto period = std::chrono::duration<double>(health_check_interval_);
  serial_health_check_timer_ = create_wall_timer(
    period, std::bind(&UARTNode::serial_health_check_timer_callback, this));
}

void UARTNode::init_uart()
{
  if (serial_mode_ == 0) return;  // 关闭模式不初始化串口
  
  // 如果已有驱动实例，先关闭
  if (uart_driver_) {
    uart_driver_->close();
  }
  
  // 创建新的驱动实例
  uart_driver_ = std::make_unique<UARTDriver>(port_name_, baudrate_);
  
  // 尝试打开串口
  if (!uart_driver_->open_port()) {
    RCLCPP_ERROR(get_logger(), "无法打开串口 %s", port_name_.c_str());
    is_healthy_ = false;
    consecutive_failure_count_++;
  } else {
    RCLCPP_INFO(get_logger(), "串口初始化成功: %s, 波特率: %d", port_name_.c_str(), baudrate_);
    is_healthy_ = true;
    consecutive_failure_count_ = 0;
    last_successful_operation_time_ = this->now();
  }
}

// 格式化字节为十六进制字符串
std::string UARTNode::format_bytes(const std::vector<uint8_t>& bytes)
{
  std::stringstream ss;
  for (size_t i = 0; i < bytes.size(); ++i) {
    ss << std::uppercase << std::setw(2) << std::setfill('0') << std::hex 
       << static_cast<int>(bytes[i]);
    if (i != bytes.size() - 1) {
      ss << " ";
    }
  }
  return ss.str();
}

void UARTNode::send_data_callback(const light_interfaces::msg::SendData::SharedPtr msg)
{
  if (serial_mode_ != 1 || !uart_driver_) {
    RCLCPP_DEBUG(get_logger(), "非实际串口模式或无串口驱动实例，不发送数据");
    return;
  }

  try {
    // 检查串口是否打开
    if (!uart_driver_->is_open()) {
      RCLCPP_ERROR(get_logger(), "串口未打开，无法发送数据");
      consecutive_failure_count_++;
      is_healthy_ = false;
      return;
    }

    // 时间戳偏移
    auto adjusted_time = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
    
    // 打包发送
    auto buffer = UARTProtocol::pack_send_data(*msg);
    ssize_t bytes_written = uart_driver_->write_data(buffer.data(), buffer.size());

    // 日志输出（受发送打印开关控制）
    if (enable_send_data_print_) {
      if (bytes_written < 0) {
        RCLCPP_ERROR(get_logger(), "发送数据失败");
        consecutive_failure_count_++;
        is_healthy_ = false;
      } else if (static_cast<size_t>(bytes_written) != buffer.size()) {
        RCLCPP_WARN(get_logger(), "部分数据发送失败，期望 %zu 字节，实际 %zd 字节",
                    buffer.size(), bytes_written);
        consecutive_failure_count_++;
        is_healthy_ = false;
      } else {
        RCLCPP_INFO(get_logger(), "发送数据 [时间戳: %.6f]: x_diff=%.3f, y_diff=%.3f, mode=%d, need_back=%d",
                    adjusted_time.nanoseconds() / 1e9,
                    msg->x_diff, msg->y_diff, msg->mode, msg->need_back);
        RCLCPP_INFO(get_logger(), "发送原始字节: %s", format_bytes(buffer).c_str());
        
        // 发送成功，重置失败计数
        consecutive_failure_count_ = 0;
        is_healthy_ = true;
        last_successful_operation_time_ = this->now();
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "发送数据回调异常: %s", e.what());
    consecutive_failure_count_++;
    is_healthy_ = false;
  }
}

void UARTNode::virtual_serial_timer_callback()
{
  if (serial_mode_ != 2 || !uart_driver_) {
    return;
  }

  try {
    // 检查串口是否打开
    if (!uart_driver_->is_open()) {
      RCLCPP_ERROR(get_logger(), "虚拟串口未打开，无法发送数据");
      consecutive_failure_count_++;
      is_healthy_ = false;
      return;
    }

    // 构造虚拟数据
    light_interfaces::msg::SendData virtual_msg;
    virtual_msg.header.stamp = this->now();
    virtual_msg.x_diff = virtual_x_diff_;
    virtual_msg.y_diff = virtual_y_diff_;
    virtual_msg.mode = static_cast<uint8_t>(virtual_mode_);
    virtual_msg.need_back = static_cast<uint8_t>(virtual_need_back_);

    // 发送数据
    auto buffer = UARTProtocol::pack_send_data(virtual_msg);
    ssize_t bytes_written = uart_driver_->write_data(buffer.data(), buffer.size());

    // 日志输出（受发送打印开关控制）
    if (enable_send_data_print_) {
      if (bytes_written < 0) {
        RCLCPP_ERROR(get_logger(), "虚拟串口发送失败");
        consecutive_failure_count_++;
        is_healthy_ = false;
      } else {
        RCLCPP_INFO(get_logger(), "虚拟串口发送: x_diff=%.3f, y_diff=%.3f, mode=%d, need_back=%d",
                    virtual_x_diff_, virtual_y_diff_, virtual_mode_, virtual_need_back_);
        RCLCPP_INFO(get_logger(), "虚拟发送原始字节: %s", format_bytes(buffer).c_str());
        
        // 发送成功，重置失败计数
        consecutive_failure_count_ = 0;
        is_healthy_ = true;
        last_successful_operation_time_ = this->now();
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "虚拟串口发送异常: %s", e.what());
    consecutive_failure_count_++;
    is_healthy_ = false;
  }
}

void UARTNode::serial_health_check_timer_callback()
{
  // 关闭模式或禁用自动重启不进行健康检查和重启
  if (serial_mode_ == 0 || !enable_auto_restart_) {
    return;
  }

  // 检查串口是否存在且已打开
  bool is_open = uart_driver_ && uart_driver_->is_open();
  
  // 检查是否超过最大重启次数
  if (max_restart_attempts_ > 0 && total_restart_attempts_ >= max_restart_attempts_) {
    RCLCPP_FATAL(get_logger(), "已达到最大重启次数 (%d)，请手动检查串口", max_restart_attempts_);
    return;
  }

  // 检查健康状态
  if (!is_open || !is_healthy_ || consecutive_failure_count_ >= max_failure_count_) {
    rclcpp::Duration time_since_last_restart = this->now() - last_restart_time_;
    
    // 检查是否在冷却期内
    if (time_since_last_restart.seconds() < restart_cooldown_) {
      RCLCPP_WARN(get_logger(), "串口不健康，但处于重启冷却期 (%d/%d 失败)，剩余冷却时间: %.1fs",
                  consecutive_failure_count_, max_failure_count_,
                  restart_cooldown_ - time_since_last_restart.seconds());
      return;
    }
    
    // 尝试重启串口
    RCLCPP_WARN(get_logger(), "串口不健康，尝试重启 (%d/%d 失败, 总重启次数: %d/%d)",
                consecutive_failure_count_, max_failure_count_,
                total_restart_attempts_, max_restart_attempts_);
    
    if (restart_serial()) {
      RCLCPP_INFO(get_logger(), "串口重启成功");
    } else {
      RCLCPP_ERROR(get_logger(), "串口重启失败");
    }
  } else {
    // 串口状态正常
    if (enable_data_print_) {
      RCLCPP_DEBUG(get_logger(), "串口状态正常");
    }
  }
}

bool UARTNode::restart_serial()
{
  // 如果禁用自动重启，直接返回失败
  if (!enable_auto_restart_) {
    RCLCPP_WARN(get_logger(), "自动重启功能已禁用，不执行重启操作");
    return false;
  }
  
  // 记录重启时间
  last_restart_time_ = this->now();
  total_restart_attempts_++;
  
  try {
    // 重启前延迟
    if (restart_delay_ > 0) {
      RCLCPP_INFO(get_logger(), "重启前延迟 %d 毫秒...", restart_delay_);
      std::this_thread::sleep_for(std::chrono::milliseconds(restart_delay_));
    }
    
    // 关闭并重新初始化串口
    RCLCPP_INFO(get_logger(), "重启串口: %s @ %d", port_name_.c_str(), baudrate_);
    
    // 销毁并重新创建UART驱动
    init_uart();
    
    // 检查是否成功
    if (uart_driver_ && uart_driver_->is_open() && is_healthy_) {
      // 重启成功，重置状态变量
      consecutive_failure_count_ = 0;
      return true;
    }
    
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "重启串口时发生异常: %s", e.what());
    return false;
  }
}

}  // namespace light_serial

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(light_serial::UARTNode)
