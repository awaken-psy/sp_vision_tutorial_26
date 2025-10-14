// 防止头文件重复包含
#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
// 控制命令结构体：定义发送给机器人执行机构的控制指令
// 用于自动瞄准系统的输出控制
struct Command
{
  bool control;               // 控制使能标志：true表示启用控制，false表示禁用
  bool shoot;                 // 射击命令：true表示发射，false表示停止
  double yaw;                 // 偏航角控制量：云台水平旋转角度（单位：弧度或度）
  double pitch;               // 俯仰角控制量：云台垂直旋转角度（单位：弧度或度）
  double horizon_distance = 0;  // 水平距离：无人机专用参数，目标在水平面上的距离（单位：米）
};

}  // namespace io

#endif  // IO__COMMAND_HPP