#ifndef LIGHT_SERIAL_UART_DRIVER_HPP_
#define LIGHT_SERIAL_UART_DRIVER_HPP_

#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

namespace light_serial
{

class UARTDriver
{
public:
  explicit UARTDriver(const std::string & port_name, int baudrate);
  ~UARTDriver();

  // 将open()重命名为open_port()以避免与系统函数冲突
  bool open_port();
  void close();
  bool is_open() const;

  ssize_t read_data(uint8_t * buffer, size_t size);
  ssize_t write_data(const uint8_t * buffer, size_t size);

private:
  std::string port_name_;
  int baudrate_;
  int fd_;
  bool is_open_;
};

}  // namespace light_serial

#endif  // LIGHT_SERIAL_UART_DRIVER_HPP_
