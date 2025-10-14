// 防止头文件重复包含
#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>           // 时间库
#include <memory>           // 智能指针
#include <opencv2/opencv.hpp>  // OpenCV计算机视觉库
#include <string>

namespace io
{
// 相机基类：定义统一的相机接口
// 使用桥接模式支持不同类型的相机实现
class CameraBase
{
public:
  // 虚析构函数确保派生类正确释放资源
  virtual ~CameraBase() = default;
  
  // 纯虚函数：读取图像和时间戳
  // img: 输出的图像数据
  // timestamp: 图像采集的时间戳
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

// 相机封装类：提供统一的相机操作接口
// 隐藏具体相机实现的细节
class Camera
{
public:
  // 构造函数：通过配置文件路径初始化相机
  // config_path: 相机配置文件路径
  Camera(const std::string & config_path);
  
  // 读取图像和时间戳
  // img: 输出的图像数据
  // timestamp: 图像采集的时间戳
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  // 相机实现类的智能指针，支持多态（可能是USBCamera、MVCamera等）
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP