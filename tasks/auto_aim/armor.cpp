#include "armor.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace auto_aim
{
// Lightbar类的构造函数：通过旋转矩形初始化灯条属性
Lightbar::Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id)
: id(id), rotated_rect(rotated_rect)  // 初始化ID和旋转矩形
{
  // 获取旋转矩形的四个角点
  std::vector<cv::Point2f> corners(4);
  rotated_rect.points(&corners[0]);

  // 按y坐标（垂直方向）对角点进行排序，找到最上面和最下面的点
  std::sort(corners.begin(), corners.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;  // 按y坐标升序排列
  });

  // 计算灯条的中心点（旋转矩形的中心）
  center = rotated_rect.center;

  // 计算灯条的顶部点：取最上面两个点的中点
  top = (corners[0] + corners[1]) / 2;

  // 计算灯条的底部点：取最下面两个点的中点
  bottom = (corners[2] + corners[3]) / 2;

  // 计算从顶部到底部的向量
  // NOTES:图像坐标系通常以左上角为原点(0,0)，x轴水平向右，y轴垂直向下。所以无论如何，top2bottom的y轴坐标为正
  top2bottom = bottom - top;

  // 将顶部和底部点存入点集
  points.emplace_back(top);
  points.emplace_back(bottom);

  // 计算灯条宽度：最上面两个点之间的距离
  width = cv::norm(corners[0] - corners[1]);

  // 计算灯条角度：使用atan2计算top2bottom向量与x轴的夹角
  // atan2范围：[-PI,PI]，但实际只能为[0,PI]
  angle = std::atan2(top2bottom.y, top2bottom.x);

  // 计算角度误差：灯条实际角度与垂直方向（π/2）的绝对差值
  // 用于判断灯条是否接近垂直状态
  angle_error = std::abs(angle - CV_PI / 2);

  // 计算灯条长度：顶部到底部的距离
  length = cv::norm(top2bottom);

  // 计算长宽比：长度/宽度，用于筛选合适的灯条
  ratio = length / width;
}

// Armor类的传统构造函数：通过左右灯条构建装甲板
Armor::Armor(const Lightbar & left, const Lightbar & right)
: left(left), right(right), duplicated(false)  // 初始化左右灯条，标记为非重复装甲板
{
  // 继承左侧灯条的颜色作为装甲板颜色
  color = left.color;

  //QUESTION
  // 计算装甲板中心点：左右灯条中心点的中点
  center = (left.center + right.center) / 2;

  // 构建装甲板的四个角点（顺时针顺序）
  points.emplace_back(left.top);      // 左上角
  points.emplace_back(right.top);     // 右上角
  points.emplace_back(right.bottom);  // 右下角
  points.emplace_back(left.bottom);   // 左下角

  // 计算左右灯条中心点之间的向量和距离（装甲板宽度）
  auto left2right = right.center - left.center;
  auto width = cv::norm(left2right);

  // 获取左右灯条的最大长度和最小长度
  auto max_lightbar_length = std::max(left.length, right.length);
  auto min_lightbar_length = std::min(left.length, right.length);

  // 计算装甲板的长宽比：宽度/最大灯条长度
  // NOTES: 这个比值用于筛选合理的装甲板（排除过于细长或过于扁平的误检测）
  ratio = width / max_lightbar_length;

  // 计算灯条长度比例：最长灯条/最短灯条（用于评估灯条匹配度）
  // NOTES: 如果 side_ratio 过大，说明左右灯条长度差异太大，可能是误匹配
  side_ratio = max_lightbar_length / min_lightbar_length;

  // 计算装甲板的旋转角度（相对于水平方向）
  auto roll = std::atan2(left2right.y, left2right.x);

  // 计算左右灯条与装甲板方向的矩形误差
  // NOTES: 检查灯条方向与装甲板方向的垂直关系，理想情况下，灯条应该垂直于装甲板连接线
  auto left_rectangular_error = std::abs(left.angle - roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(right.angle - roll - CV_PI / 2);

  // 取左右灯条矩形误差的最大值作为装甲板的矩形误差
  // NOTES: 确保装甲板的两个灯条都满足垂直关系，如果有一个灯条明显不垂直，整个装甲板就应该被排除
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);
}

//神经网络构造函数
Armor::Armor(
  int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints)
: class_id(class_id), confidence(confidence), box(box), points(armor_keypoints)
{
  center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
  auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);
  auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);
  auto max_width = std::max(left_width, right_width);
  auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);
  auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);
  auto max_length = std::max(top_length, bottom_length);
  auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
  auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
  auto left2right = right_center - left_center;
  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
    roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
    roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

  ratio = max_length / max_width;
  // color = class_id == 0 ? Color::blue : Color::red;

  if (class_id >= 0 && class_id < armor_properties.size()) {
    auto [color, name, type] = armor_properties[class_id];
    this->color = color;
    this->name = name;
    this->type = type;
  } else {
    this->color = blue;      // Default
    this->name = not_armor;  // Default
    this->type = small;      // Default
  }
}

//神经网络ROI构造函数
Armor::Armor(
  int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
  cv::Point2f offset)
: class_id(class_id), confidence(confidence), box(box), points(armor_keypoints)
{
  std::transform(
    armor_keypoints.begin(), armor_keypoints.end(), armor_keypoints.begin(),
    [&offset](const cv::Point2f & point) { return point + offset; });
  std::transform(
    points.begin(), points.end(), points.begin(),
    [&offset](const cv::Point2f & point) { return point + offset; });
  center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
  auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);
  auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);
  auto max_width = std::max(left_width, right_width);
  auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);
  auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);
  auto max_length = std::max(top_length, bottom_length);
  auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
  auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
  auto left2right = right_center - left_center;
  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
    roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
    roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

  ratio = max_length / max_width;
  // color = class_id == 0 ? Color::blue : Color::red;

  if (class_id >= 0 && class_id < armor_properties.size()) {
    auto [color, name, type] = armor_properties[class_id];
    this->color = color;
    this->name = name;
    this->type = type;
  } else {
    this->color = blue;      // Default
    this->name = not_armor;  // Default
    this->type = small;      // Default
  }
}

// YOLOV5构造函数
Armor::Armor(
  int color_id, int num_id, float confidence, const cv::Rect & box,
  std::vector<cv::Point2f> armor_keypoints)
: confidence(confidence), box(box), points(armor_keypoints)
{
  center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
  auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);
  auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);
  auto max_width = std::max(left_width, right_width);
  auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);
  auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);
  auto max_length = std::max(top_length, bottom_length);
  auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
  auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
  auto left2right = right_center - left_center;
  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
    roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
    roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

  ratio = max_length / max_width;
  color = color_id == 0 ? Color::blue : color_id == 1 ? Color::red : Color::extinguish;
  name = num_id == 0  ? ArmorName::sentry
         : num_id > 5 ? ArmorName(num_id)
                      : ArmorName(num_id - 1);  //TODO 考虑Bb
  type = num_id == 1 ? ArmorType::big : ArmorType::small;
}

// YOLOV5+ROI构造函数
Armor::Armor(
  int color_id, int num_id, float confidence, const cv::Rect & box,
  std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset)
: confidence(confidence), box(box), points(armor_keypoints)
{
  std::transform(
    armor_keypoints.begin(), armor_keypoints.end(), armor_keypoints.begin(),
    [&offset](const cv::Point2f & point) { return point + offset; });
  std::transform(
    points.begin(), points.end(), points.begin(),
    [&offset](const cv::Point2f & point) { return point + offset; });
  center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
  auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);
  auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);
  auto max_width = std::max(left_width, right_width);
  auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);
  auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);
  auto max_length = std::max(top_length, bottom_length);
  auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
  auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
  auto left2right = right_center - left_center;
  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
    roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(
    std::atan2(
      (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
    roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

  ratio = max_length / max_width;
  color = color_id == 0 ? Color::blue : color_id == 1 ? Color::red : Color::extinguish;
  name = num_id == 0 ? ArmorName::sentry : num_id > 5 ? ArmorName(num_id) : ArmorName(num_id - 1);
  type = num_id == 1 ? ArmorType::big : ArmorType::small;
}

}  // namespace auto_aim