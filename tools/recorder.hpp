// 防止头文件重复包含
#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>  // 四元数等几何数据类型
#include <chrono>          // 时间库
#include <fstream>         // 文件流操作
#include <opencv2/opencv.hpp>  // OpenCV计算机视觉库
#include <thread>          // 线程支持

#include "tools/thread_safe_queue.hpp"  // 线程安全队列

namespace tools
{
// 数据记录器类：用于同步记录视频帧和对应的姿态数据
// 适用于机器人视觉系统的数据采集和实验记录
class Recorder
{
public:
  // 构造函数：设置记录帧率
  Recorder(double fps = 30);
  // 析构函数：确保资源正确释放
  ~Recorder();
  
  // 记录一帧数据：图像、姿态四元数、时间戳
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);

private:
  // 帧数据结构体：包含图像、姿态和时间戳
  struct FrameData
  {
    cv::Mat img;                                  // 图像数据
    Eigen::Quaterniond q;                         // 姿态四元数
    std::chrono::steady_clock::time_point timestamp;  // 时间戳
  };
  
  bool init_;                    // 初始化标志
  std::atomic<bool> stop_thread_; // 线程停止标志（原子操作）
  double fps_;                   // 记录帧率
  std::string text_path_;        // 文本数据保存路径
  std::string video_path_;       // 视频文件保存路径
  std::ofstream text_writer_;    // 文本文件写入器
  cv::VideoWriter video_writer_; // 视频文件写入器
  std::chrono::steady_clock::time_point start_time_;  // 记录开始时间
  std::chrono::steady_clock::time_point last_time_;   // 上一帧记录时间
  tools::ThreadSafeQueue<FrameData> queue_;  // 线程安全队列，存储待处理的帧数据
  std::thread saving_thread_;  // 负责保存帧数据的后台线程
  
  // 初始化记录器：根据第一帧图像设置视频编码参数
  void init(const cv::Mat & img);
  // 后台线程函数：从队列中取出数据并保存到文件
  void save_to_file();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP