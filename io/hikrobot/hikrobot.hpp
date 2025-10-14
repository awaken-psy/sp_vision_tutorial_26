// 防止头文件重复包含
#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>           // 原子操作
#include <chrono>           // 时间库
#include <opencv2/opencv.hpp>  // OpenCV计算机视觉库
#include <string>
#include <thread>           // 线程支持

#include "MvCameraControl.h"  // 海康威视相机控制SDK
#include "io/camera.hpp"      // 相机基类
#include "tools/thread_safe_queue.hpp"  // 线程安全队列

namespace io
{
// 海康威视工业相机类：继承自CameraBase
// 封装海康威视相机的控制和图像采集功能
class HikRobot : public CameraBase
{
public:
  // 构造函数：设置相机参数
  // exposure_ms: 曝光时间（毫秒）
  // gain: 增益值
  // vid_pid: USB设备的厂商ID和产品ID（用于识别特定相机）
  HikRobot(double exposure_ms, double gain, const std::string & vid_pid);
  
  // 析构函数：清理资源，停止所有线程
  ~HikRobot() override;
  
  // 从相机读取图像和时间戳（重写基类虚函数）
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  // 相机数据结构体：存储图像和对应的时间戳
  struct CameraData
  {
    cv::Mat img;                                  // 图像数据
    std::chrono::steady_clock::time_point timestamp;  // 时间戳
  };

  double exposure_us_;      // 曝光时间（微秒）
  double gain_;             // 增益值

  std::thread daemon_thread_;    // 守护线程（监控相机状态）
  std::atomic<bool> daemon_quit_; // 守护线程退出标志

  void * handle_;                 // 海康相机句柄
  std::thread capture_thread_;    // 图像采集线程
  std::atomic<bool> capturing_;   // 采集状态标志
  std::atomic<bool> capture_quit_; // 采集线程退出标志
  tools::ThreadSafeQueue<CameraData> queue_;  // 线程安全队列，存储采集的图像数据

  int vid_, pid_;  // USB设备的厂商ID和产品ID

  // 开始图像采集
  void capture_start();
  // 停止图像采集
  void capture_stop();

  // 设置相机浮点型参数
  void set_float_value(const std::string & name, double value);
  // 设置相机枚举型参数
  void set_enum_value(const std::string & name, unsigned int value);

  // 从字符串解析厂商ID和产品ID
  void set_vid_pid(const std::string & vid_pid);
  // 重置USB设备（解决连接问题）
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP