// 防止头文件重复包含
#ifndef AUTO_AIM__YOLOV5_HPP
#define AUTO_AIM__YOLOV5_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

// 引入自定义的头文件
#include "tasks/auto_aim/armor.hpp"  // 装甲板数据结构
#include "tasks/auto_aim/yolo.hpp"   // YOLO基类

namespace auto_aim
{
// YOLOV5类，继承自YOLOBase基类
class YOLOV5 : public YOLOBase
{
public:
  // 构造函数：接收配置文件路径和调试标志
  YOLOV5(const std::string & config_path, bool debug);

  // 重写基类的检测函数：对输入的BGR图像进行装甲板检测
  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  // 重写基类的后处理函数：处理模型输出，生成装甲板列表
  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  // OpenVINO相关变量
  std::string device_;          // 推理设备（如CPU、GPU等）
  std::string model_path_;      // 模型文件路径
  std::string save_path_;       // 保存路径
  std::string debug_path_;      // 调试文件路径
  bool debug_;                  // 是否开启调试模式
  bool use_roi_;                // 是否使用感兴趣区域(ROI)

  // 模型参数
  const int class_num_ = 13;          // 类别数量（13种装甲板类型）
  const float nms_threshold_ = 0.3;   // 非极大值抑制阈值
  const float score_threshold_ = 0.7; // 得分阈值
  double min_confidence_;             // 最小置信度
  double binary_threshold_;           // 二值化阈值

  // OpenVINO推理引擎相关对象
  ov::Core core_;                      // OpenVINO核心对象
  ov::CompiledModel compiled_model_;   // 编译后的模型

  // 图像处理相关变量
  cv::Rect roi_;              // 感兴趣区域
  cv::Point2f offset_;        // 坐标偏移量
  cv::Mat tmp_img_;           // 临时图像存储

  // 声明友元类，允许MultiThreadDetector访问私有成员
  friend class MultiThreadDetector;

  // 检查装甲板名称是否有效
  bool check_name(const Armor & armor) const;
  // 检查装甲板类型是否有效
  bool check_type(const Armor & armor) const;

  // 获取归一化的中心坐标（相对于图像尺寸）
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  // 解析模型输出，生成装甲板列表
  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  // 保存检测结果（调试用）
  void save(const Armor & armor) const;
  // 在图像上绘制检测结果（调试用）
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  // Sigmoid激活函数
  double sigmoid(double x);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_HPP