// 防止头文件重复包含
#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>          // 线性代数库，用于矩阵运算
#include <opencv2/opencv.hpp>   // OpenCV计算机视觉库
#include <string>
#include <vector>

namespace auto_aim
{
// 装甲板颜色枚举
enum Color
{
  red,         // 红色
  blue,        // 蓝色
  extinguish,  // 熄灭状态
  purple       // 紫色
};
// 颜色字符串表示
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

// 装甲板类型枚举（大小）
enum ArmorType
{
  big,   // 大装甲板
  small  // 小装甲板
};
// 装甲板类型字符串表示
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

// 装甲板名称枚举（编号）
enum ArmorName
{
  one,       // 1号
  two,       // 2号  
  three,     // 3号
  four,      // 4号
  five,      // 5号
  sentry,    // 哨兵
  outpost,   // 前哨站
  base,      // 基地
  not_armor  // 非装甲板
};
// 装甲板名称字符串表示
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

// 装甲板优先级枚举
enum ArmorPriority
{
  first = 1,  // 第一优先级
  second,     // 第二优先级
  third,      // 第三优先级
  forth,      // 第四优先级
  fifth       // 第五优先级
};

// clang-format off
// 装甲板属性元组列表：定义所有可能的装甲板组合（颜色、名称、类型）
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

// 灯条结构体定义
struct Lightbar
{
  std::size_t id;                    // 灯条ID
  Color color;                       // 灯条颜色
  cv::Point2f center, top, bottom;   // 中心点、顶部点、底部点坐标
  cv::Point2f top2bottom;            // 从顶部到底部的向量
  std::vector<cv::Point2f> points;   // 灯条轮廓点集
  double angle, angle_error;         // 角度、角度误差
  double length, width, ratio;       // 长度、宽度、长宽比
  cv::RotatedRect rotated_rect;      // 旋转矩形框

  // 构造函数：通过旋转矩形和ID初始化灯条
  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  // 默认构造函数
  Lightbar() {};
};

// 装甲板结构体定义
struct Armor
{
  Color color;                // 装甲板颜色
  Lightbar left, right;       // 左右灯条（原注释：used to be const）
  cv::Point2f center;         // 中心点（注意：不是对角线交点，不能作为实际中心！）
  cv::Point2f center_norm;    // 归一化坐标
  std::vector<cv::Point2f> points;  // 装甲板角点

  double ratio;               // 两灯条中点连线与长灯条长度之比
  double side_ratio;          // 长灯条与短灯条长度之比
  double rectangular_error;   // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type;             // 装甲板类型（大小）
  ArmorName name;             // 装甲板名称（编号）
  ArmorPriority priority;     // 优先级
  int class_id;               // 类别ID
  cv::Rect box;               // 边界框
  cv::Mat pattern;            // 装甲板图像模式
  double confidence;          // 置信度
  bool duplicated;            // 是否重复检测

  // 3D坐标信息
  Eigen::Vector3d xyz_in_gimbal;  // 在云台坐标系中的位置（单位：米）
  Eigen::Vector3d xyz_in_world;   // 在世界坐标系中的位置（单位：米）
  Eigen::Vector3d ypr_in_gimbal;  // 在云台坐标系中的偏航-俯仰-横滚角（单位：弧度）
  Eigen::Vector3d ypr_in_world;   // 在世界坐标系中的偏航-俯仰-横滚角（单位：弧度）
  Eigen::Vector3d ypd_in_world;   // 在世界坐标系中的球坐标（偏航-俯仰-距离）

  double yaw_raw;  // 原始偏航角（单位：弧度）

  // 多个构造函数重载，支持不同的初始化方式
  Armor(const Lightbar & left, const Lightbar & right);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP