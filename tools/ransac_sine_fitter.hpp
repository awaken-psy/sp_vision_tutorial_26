#pragma once  // 跨平台编译器指令，防止头文件重复包含

#include <Eigen/Dense>  // 线性代数库
#include <deque>
#include <iostream>
#include <random>       // 随机数生成
#include <vector>

namespace tools
{

// RANSAC正弦曲线拟合器类
// 使用RANSAC（随机抽样一致性）算法从带噪声的数据中拟合正弦曲线
class RansacSineFitter
{
public:
  // 拟合结果结构体
  struct Result
  {
    double A = 0.0;      // 振幅（Amplitude）
    double omega = 0.0;  // 角频率（Angular frequency）
    double phi = 0.0;    // 相位（Phase）
    double C = 0.0;      // 垂直偏移（Vertical offset）
    int inliers = 0;     // 内点数量（符合模型的数据点数量）
  };
  Result best_result_;   // 最佳拟合结果

  // 构造函数：设置RANSAC参数
  RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega);

  // 添加数据点：时间t和对应的值v
  void add_data(double t, double v);

  // 执行RANSAC拟合
  void fit();

  // 正弦函数定义：计算给定参数下时间t对应的值
  double sine_function(double t, double A, double omega, double phi, double C)
  {
    return A * std::sin(omega * t + phi) + C;
  }

private:
  int max_iterations_;   // RANSAC最大迭代次数
  double threshold_;     // 内点阈值（判断数据点是否符合模型的误差阈值）
  double min_omega_;     // 最小角频率限制
  double max_omega_;     // 最大角频率限制
  std::mt19937 gen_;     // 梅森旋转算法随机数生成器
  std::deque<std::pair<double, double>> fit_data_;  // 存储待拟合的数据（时间，值）

  // 使用部分数据点拟合正弦模型（固定角频率）
  bool fit_partial_model(
    const std::vector<std::pair<double, double>> & sample, double omega, Eigen::Vector3d & params);

  // 评估当前模型参数下的内点数量
  int evaluate_inliers(double A, double omega, double phi, double C);
};

}  // namespace tools