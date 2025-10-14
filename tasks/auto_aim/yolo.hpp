// 防止头文件重复包含
#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include "armor.hpp"  // 装甲板数据结构

namespace auto_aim
{
// YOLO检测器基类：定义统一的检测接口
class YOLOBase
{
public:
  // 纯虚函数：检测接口，需要在派生类中实现
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  // 纯虚函数：后处理接口，需要在派生类中实现
  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

// YOLO检测器封装类：提供统一的YOLO检测接口
class YOLO
{
public:
  // 构造函数：通过配置文件路径和调试标志初始化
  YOLO(const std::string & config_path, bool debug = true);

  // 检测函数：对输入图像进行装甲板检测
  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  // 后处理函数：处理模型输出，生成装甲板列表
  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  // YOLO实现类的智能指针，支持多态（可能是YOLOV5、YOLOV8等）
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP