// 防止头文件重复包含
#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"  // 扩展卡尔曼滤波器

namespace auto_aim
{

// 目标跟踪类：用于跟踪单个机器人目标（包含多个装甲板）
class Target
{
public:
  ArmorName name;           // 目标名称（装甲板编号）
  ArmorType armor_type;     // 装甲板类型（大小）
  ArmorPriority priority;   // 优先级
  bool jumped;              // 是否发生跳变（目标突然移动）
  int last_id;              // 上一次更新的装甲板ID（仅用于调试）

  // 默认构造函数
  Target() = default;
  // 构造函数：通过第一个检测到的装甲板初始化目标
  Target(
    const Armor & armor,                      // 第一个检测到的装甲板，用于初始化目标状态
    std::chrono::steady_clock::time_point t,  // 当前时间戳，用于初始化滤波器时间
    Eigen::VectorXd P0_dig,                   // 初始状态协方差矩阵的对角线元素
    double radius = 0.2,                      // 机器人装甲板的分布半径（单位：米），默认0.2m
    int armor_num = 4);                       // 机器人的装甲板数量，默认4个

  // 预测函数：根据时间点预测目标状态
  void predict(std::chrono::steady_clock::time_point t);
  // 预测函数：根据时间间隔预测目标状态
  void predict(double dt);
  // 更新函数：使用新的装甲板检测更新目标状态
  void update(const Armor & armor);

  // 获取EKF状态向量
  Eigen::VectorXd ekf_x() const;
  // 获取EKF滤波器对象
  const tools::ExtendedKalmanFilter & ekf() const;
  // 获取所有装甲板在世界坐标系中的位置和朝向列表
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  // 判断EKF是否发散
  bool diverged() const;
  // 判断EKF是否收敛
  bool convergened();

  bool isinit = false;      // 初始化标志

  // 检查是否完成初始化
  bool checkinit();

private:
  int armor_num_;           // 装甲板数量（通常为4或5）
  int switch_count_;        // 切换计数（装甲板切换次数）
  int update_count_;        // 更新计数

  bool is_switch_;          // 是否正在切换装甲板
  bool is_converged_;       // 是否已收敛

  tools::ExtendedKalmanFilter ekf_;  // 扩展卡尔曼滤波器
  std::chrono::steady_clock::time_point t_;  // 上一次更新时间点

  // 使用偏航-俯仰-距离-角度（YPDA）方法更新EKF
  void update_ypda(const Armor & armor, int id);

  // 观测函数：计算指定ID装甲板在相机坐标系中的3D位置
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  // 计算观测函数的雅可比矩阵
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP