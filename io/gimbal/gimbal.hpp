// 防止头文件重复包含
#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>  // 四元数等几何类型
#include <atomic>          // 原子操作
#include <chrono>          // 时间库
#include <mutex>           // 互斥锁
#include <string>
#include <thread>          // 线程支持
#include <tuple>           // 元组

#include "serial/serial.h"  // 串口通信库
#include "tools/thread_safe_queue.hpp"  // 线程安全队列

namespace io
{
// 从云台接收的数据结构体（打包对齐，避免内存对齐填充）
// 用于接收云台状态信息
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};  // 数据包头，固定为"SP"
  uint8_t mode;  // 云台模式：0-空闲, 1-自瞄, 2-小符, 3-大符
  float q[4];    // 四元数姿态，wxyz顺序
  float yaw;     // 当前偏航角
  float yaw_vel; // 偏航角速度
  float pitch;   // 当前俯仰角
  float pitch_vel; // 俯仰角速度
  float bullet_speed; // 子弹速度
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;         // CRC16校验值
};

// 编译时检查结构体大小，确保不超过64字节
static_assert(sizeof(GimbalToVision) <= 64);

// 发送到云台的数据结构体（打包对齐）
// 用于发送视觉系统的控制指令
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};  // 数据包头，固定为"SP"
  uint8_t mode;  // 控制模式：0-不控制, 1-控制云台但不开火，2-控制云台且开火
  float yaw;     // 目标偏航角
  float yaw_vel; // 目标偏航角速度
  float yaw_acc; // 目标偏航角加速度
  float pitch;   // 目标俯仰角
  float pitch_vel; // 目标俯仰角速度
  float pitch_acc; // 目标俯仰角加速度
  uint16_t crc16;   // CRC16校验值
};

// 编译时检查结构体大小，确保不超过64字节
static_assert(sizeof(VisionToGimbal) <= 64);

// 云台工作模式枚举
enum class GimbalMode
{
  IDLE,        // 空闲模式
  AUTO_AIM,    // 自动瞄准模式
};

// 云台状态结构体
struct GimbalState
{
  float yaw;           // 偏航角
  float yaw_vel;       // 偏航角速度
  float pitch;         // 俯仰角
  float pitch_vel;     // 俯仰角速度
  float bullet_speed;  // 子弹速度
  uint16_t bullet_count; // 子弹计数
};

// 云台控制类：管理与云台的串口通信
// 负责接收云台状态和发送控制指令
class Gimbal
{
public:
  // 构造函数：通过配置文件路径初始化云台
  Gimbal(const std::string & config_path);

  // 析构函数：停止线程并清理资源
  ~Gimbal();

  // 获取当前云台模式
  GimbalMode mode() const;
  // 获取当前云台状态
  GimbalState state() const;
  // 将云台模式转换为字符串表示
  std::string str(GimbalMode mode) const;
  // 获取指定时间点的姿态四元数（通过插值）
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  // 发送控制指令到云台
  // control: 是否启用控制
  // fire: 是否开火
  // yaw: 目标偏航角
  // pitch: 目标俯仰角
  void send(
    bool control, bool fire, float yaw, float pitch);

private:
  serial::Serial serial_;  // 串口对象

  std::thread thread_;     // 数据读取线程
  std::atomic<bool> quit_ = false;  // 线程退出标志
  mutable std::mutex mutex_;        // 互斥锁，保护共享数据

  GimbalToVision rx_data_;  // 接收数据缓冲区
  VisionToGimbal tx_data_;  // 发送数据缓冲区

  GimbalMode mode_ = GimbalMode::IDLE;  // 当前云台模式
  GimbalState state_;                   // 当前云台状态
  // 姿态四元数和时间戳队列，用于历史数据记录和插值
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  // 从串口读取数据
  bool read(uint8_t * buffer, size_t size);
  // 数据读取线程函数
  void read_thread();
  // 重新连接串口
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP