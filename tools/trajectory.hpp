// 防止头文件重复包含
#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
// 弹道计算结构体：用于计算抛射体运动轨迹参数
// 基于理想抛射体运动模型，不考虑空气阻力
struct Trajectory
{
  bool unsolvable;    // 是否无解标志（当目标无法命中时设为true）
  double fly_time;    // 飞行时间，单位：秒
  double pitch;       // 发射仰角，抬头为正，单位：弧度

  // 构造函数：根据初始条件和目标位置计算弹道参数
  // v0: 子弹初速度大小，单位：m/s
  // d: 目标水平距离，单位：m
  // h: 目标竖直高度，单位：m
  // 基于理想抛射体运动方程计算所需的发射角度和飞行时间
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP