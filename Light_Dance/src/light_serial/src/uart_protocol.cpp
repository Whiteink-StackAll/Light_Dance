#include "light_serial/uart_protocol.hpp"
#include <cstring>

namespace light_serial
{

void UARTProtocol::float_to_bytes(float value, uint8_t* bytes)
{
  if (!bytes) return;
  
  uint32_t temp;
  std::memcpy(&temp, &value, sizeof(value));
  bytes[0] = temp & 0xFF;
  bytes[1] = (temp >> 8) & 0xFF;
  bytes[2] = (temp >> 16) & 0xFF;
  bytes[3] = (temp >> 24) & 0xFF;
}

void UARTProtocol::uint8_to_bytes(uint8_t value, uint8_t* bytes)
{
  if (!bytes) return;
  bytes[0] = value & 0xFF;
}

std::vector<uint8_t> UARTProtocol::pack_send_data(const light_interfaces::msg::SendData & msg)
{
  // 初始化数据包并全部填充0（确保空位为0）
  std::vector<uint8_t> buffer(SEND_PACKET_SIZE, 0);
  size_t index = 0;

  // 帧头
  buffer[index++] = FRAME_HEADER;

  // x轴差值（float32，4字节）
  uint8_t x_bytes[4];
  float_to_bytes(msg.x_diff, x_bytes);
  for (int i = 0; i < 4; ++i) {
    buffer[index++] = x_bytes[i];
  }

  // y轴差值（float32，4字节）
  uint8_t y_bytes[4];
  float_to_bytes(msg.y_diff, y_bytes);
  for (int i = 0; i < 4; ++i) {
    buffer[index++] = y_bytes[i];
  }

  // 模式（uint8_t，1字节）
  uint8_t mode_byte;
  uint8_to_bytes(static_cast<uint8_t>(msg.mode), &mode_byte);
  buffer[index++] = mode_byte;

  // 是否需要后退（uint8_t，1字节）
  uint8_t need_back_byte;
  uint8_to_bytes(static_cast<uint8_t>(msg.need_back), &need_back_byte);
  buffer[index++] = need_back_byte;

  // 空位（4字节，已初始化为0）
  index += 4;
  
  // 帧尾
  buffer[index] = FRAME_TAIL;

  return buffer;
}

}  // namespace light_serial
