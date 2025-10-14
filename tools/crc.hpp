// 防止头文件重复包含
#ifndef TOOLS__CRC_HPP
#define TOOLS__CRC_HPP

#include <cstdint>  // 标准整数类型

namespace tools
{
// 计算CRC8校验值
// data: 输入数据指针
// len: 数据长度（不包括CRC8字节本身）
// 返回: 计算得到的CRC8校验值
uint8_t get_crc8(const uint8_t * data, uint16_t len);

// 验证CRC8校验值
// data: 输入数据指针（包含CRC8字节）
// len: 数据长度（包括CRC8字节本身）
// 返回: 校验成功返回true，失败返回false
bool check_crc8(const uint8_t * data, uint16_t len);

// 计算CRC16校验值
// data: 输入数据指针
// len: 数据长度（不包括CRC16字节本身）
// 返回: 计算得到的CRC16校验值
uint16_t get_crc16(const uint8_t * data, uint32_t len);

// 验证CRC16校验值
// data: 输入数据指针（包含CRC16字节）
// len: 数据长度（包括CRC16字节本身）
// 返回: 校验成功返回true，失败返回false
bool check_crc16(const uint8_t * data, uint32_t len);

}  // namespace tools

#endif  // TOOLS__CRC_HPP