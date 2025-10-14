// 防止头文件重复包含
#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // socket地址结构体定义

#include <mutex>  // 互斥锁，用于线程安全
#include <nlohmann/json.hpp>  // JSON库
#include <string>

namespace tools
{
// 绘图仪类：通过网络发送JSON数据用于实时可视化
// 通常用于调试和数据显示，将数据发送到外部可视化工具
class Plotter
{
public:
  // 构造函数：指定目标主机和端口
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

  // 析构函数：清理socket资源
  ~Plotter();

  // 发送JSON数据到可视化工具
  void plot(const nlohmann::json & json);

private:
  int socket_;                    // socket文件描述符
  sockaddr_in destination_;       // 目标地址信息
  std::mutex mutex_;              // 互斥锁，确保线程安全的网络发送
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP