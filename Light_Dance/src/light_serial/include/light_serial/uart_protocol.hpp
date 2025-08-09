#ifndef LIGHT_SERIAL_UART_PROTOCOL_HPP_
#define LIGHT_SERIAL_UART_PROTOCOL_HPP_

#include "light_interfaces/msg/send_data.hpp"
#include <vector>

namespace light_serial
{

class UARTProtocol
{
public:
  // 帧头帧尾定义
  static constexpr uint8_t FRAME_HEADER = 0xFF;
  static constexpr uint8_t FRAME_TAIL = 0x0D;
  // 数据包大小计算：头(1) + x(4) + y(4) + mode(1) + need_back(1) + 空位(4) + 尾(1)
  static constexpr size_t SEND_PACKET_SIZE = 1 + 4 + 4 + 1 + 1 + 4 + 1;

  // 打包发送数据
  static std::vector<uint8_t> pack_send_data(const light_interfaces::msg::SendData & msg);

private:
  // 数据类型转换工具
  static void float_to_bytes(float value, uint8_t* bytes);
  static void uint8_to_bytes(uint8_t value, uint8_t* bytes);
};

}  // namespace light_serial

#endif  // LIGHT_SERIAL_UART_PROTOCOL_HPP_
