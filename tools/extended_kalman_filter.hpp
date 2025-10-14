// 防止头文件重复包含
#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>  // 线性代数库
#include <deque>
#include <functional>   // 函数对象支持
#include <map>

namespace tools
{
// 扩展卡尔曼滤波器（EKF）类
// 用于非线性系统的状态估计
class ExtendedKalmanFilter
{
public:
  // 公共成员变量
  Eigen::VectorXd x;  // 状态向量
  Eigen::MatrixXd P;  // 状态协方差矩阵

  // 默认构造函数
  ExtendedKalmanFilter() = default;

  // 构造函数：初始化状态向量、协方差矩阵和状态加法函数
  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  // 预测步骤：使用线性状态转移模型
  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  // 预测步骤：使用非线性状态转移函数
  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  // 更新步骤：使用线性观测模型
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  // 更新步骤：使用非线性观测函数
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  // 卡方检验相关数据存储
  std::map<std::string, double> data;
  // 最近NIS（归一化创新平方）失败记录队列
  std::deque<int> recent_nis_failures{0};
  // 滑动窗口大小
  size_t window_size = 100;
  // 上一次的NIS值
  double last_nis;

private:
  // 单位矩阵
  Eigen::MatrixXd I;
  // 状态向量加法函数（用于处理角度等特殊状态）
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;

  // NEES（归一化估计误差平方）计数
  int nees_count_ = 0;
  // NIS计数
  int nis_count_ = 0;
  // 总计数
  int total_count_ = 0;
};

}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP