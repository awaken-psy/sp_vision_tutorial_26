// 防止头文件重复包含
#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

#include <linux/can.h>    // Linux CAN总线头文件
#include <net/if.h>       // 网络接口
#include <sys/epoll.h>    // epoll I/O多路复用
#include <sys/ioctl.h>    // I/O控制
#include <unistd.h>       // POSIX标准函数

#include <chrono>
#include <cstring>
#include <functional>     // 函数对象
#include <stdexcept>      // 异常处理
#include <thread>         // 线程支持

#include "tools/logger.hpp"  // 日志工具

using namespace std::chrono_literals;  // 时间字面量

constexpr int MAX_EVENTS = 10;  // epoll最大事件数

namespace io
{
// SocketCAN类：Linux环境下CAN总线通信封装
// 提供CAN帧的发送和接收功能，支持自动重连
class SocketCAN
{
public:
  // 构造函数：初始化CAN接口和接收处理函数
  // interface: CAN接口名称（如"can0"）
  // rx_handler: CAN帧接收回调函数
  SocketCAN(const std::string & interface, std::function<void(const can_frame & frame)> rx_handler)
  : interface_(interface),
    socket_fd_(-1),
    epoll_fd_(-1),
    rx_handler_(rx_handler),
    quit_(false),
    ok_(false)
  {
    try_open();  // 尝试打开CAN接口

    // 守护线程：监控连接状态，在断开时自动重连
    daemon_thread_ = std::thread{[this] {
      while (!quit_) {
        std::this_thread::sleep_for(100ms);

        if (ok_) continue;  // 连接正常则继续等待

        if (read_thread_.joinable()) read_thread_.join();  // 等待读取线程结束

        close();    // 关闭当前连接
        try_open(); // 尝试重新打开
      }
    }};
  }

  // 析构函数：清理资源，停止所有线程
  ~SocketCAN()
  {
    quit_ = true;  // 设置退出标志
    if (daemon_thread_.joinable()) daemon_thread_.join();  // 等待守护线程结束
    if (read_thread_.joinable()) read_thread_.join();      // 等待读取线程结束
    close();  // 关闭CAN连接
    tools::logger()->info("SocketCAN destructed.");
  }

  // 发送CAN帧到总线
  void write(can_frame * frame) const
  {
    if (::write(socket_fd_, frame, sizeof(can_frame)) == -1)
      throw std::runtime_error("Unable to write!");
  }

private:
  std::string interface_;           // CAN接口名称
  int socket_fd_;                   // Socket文件描述符
  int epoll_fd_;                    // epoll文件描述符
  bool quit_;                       // 退出标志
  bool ok_;                         // 连接状态标志
  std::thread read_thread_;         // CAN帧读取线程
  std::thread daemon_thread_;       // 守护线程（负责重连）
  can_frame frame_;                 // CAN帧缓冲区
  epoll_event events_[MAX_EVENTS];  // epoll事件数组
  std::function<void(const can_frame & frame)> rx_handler_;  // 接收处理回调函数

  // 打开CAN接口并初始化
  void open()
  {
    // 创建RAW CAN socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) throw std::runtime_error("Error opening socket!");

    // 获取接口索引
    ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
      throw std::runtime_error("Error getting interface index!");

    // 绑定socket到指定接口
    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(sockaddr_can));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (sockaddr *)&addr, sizeof(sockaddr_can)) < 0) {
      ::close(socket_fd_);
      throw std::runtime_error("Error binding socket to interface!");
    }

    // 创建epoll实例用于I/O多路复用
    epoll_event ev;
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ == -1) throw std::runtime_error("Error creating epoll file descriptor!");

    // 将socket添加到epoll监控
    ev.events = EPOLLIN;  // 监控可读事件
    ev.data.fd = socket_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev))
      throw std::runtime_error("Error adding socket to epoll file descriptor!");

    // 创建CAN帧读取线程
    read_thread_ = std::thread([this]() {
      ok_ = true;  // 标记连接正常
      while (!quit_) {
        std::this_thread::sleep_for(10us);  // 短暂休眠减少CPU占用

        try {
          read();  // 读取CAN帧
        } catch (const std::exception & e) {
          tools::logger()->warn("SocketCAN::read() failed: {}", e.what());
          ok_ = false;  // 标记连接异常
          break;        // 退出读取循环
        }
      }
    });

    tools::logger()->info("SocketCAN opened.");
  }

  // 尝试打开CAN接口（异常安全版本）
  void try_open()
  {
    try {
      open();
    } catch (const std::exception & e) {
      tools::logger()->warn("SocketCAN::open() failed: {}", e.what());
    }
  }

  // 读取CAN帧
  void read()
  {
    // 等待epoll事件（超时2毫秒）
    int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
    if (num_events == -1) throw std::runtime_error("Error wating for events!");

    // 处理所有就绪的事件
    for (int i = 0; i < num_events; i++) {
      // 非阻塞方式读取CAN帧
      ssize_t num_bytes = recv(socket_fd_, &frame_, sizeof(can_frame), MSG_DONTWAIT);
      if (num_bytes == -1) throw std::runtime_error("Error reading from SocketCAN!");

      // 调用接收处理回调函数
      rx_handler_(frame_);
    }
  }

  // 关闭CAN连接
  void close()
  {
    if (socket_fd_ == -1) return;  // 已关闭则直接返回
    
    // 清理epoll监控
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, NULL);
    ::close(epoll_fd_);   // 关闭epoll文件描述符
    ::close(socket_fd_);  // 关闭socket文件描述符
  }
};

}  // namespace io

#endif  // IO__SOCKETCAN_HPP