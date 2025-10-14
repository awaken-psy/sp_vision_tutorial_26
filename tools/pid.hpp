// 防止头文件重复包含
#ifndef TOOLS__PID_HPP
#define TOOLS__PID_HPP

namespace tools
{
// PID控制器类：实现比例-积分-微分控制算法
class PID
{
public:
  // 构造函数：初始化PID参数
  // dt: 控制周期, 单位: s
  // kp: P项系数（比例项）
  // ki: I项系数（积分项）  
  // kd: D项系数（微分项）
  // max_out: PID最大输出值（输出限幅）
  // max_iout: I项最大输出值（积分限幅）
  // angular: 是否为角度控制（处理角度环绕问题）
  PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular = false);

  // 调试用输出变量
  float pout = 0.0f;  // P项输出, 用于调试
  float iout = 0.0f;  // I项输出, 用于调试  
  float dout = 0.0f;  // D项输出, 用于调试

  // 计算PID输出值
  // set: 目标值（设定点）
  // fdb: 反馈值（feedback）
  float calc(float set, float fdb);

private:
  const float dt_;        // 控制周期（秒）
  const float kp_, ki_, kd_;  // PID系数
  const float max_out_, max_iout_;  // 输出限幅值
  const bool angular_;    // 角度控制标志（处理360°环绕）

  float last_fdb_ = 0.0f;  // 上次反馈值（用于微分计算）
};

}  // namespace tools

#endif  // TOOLS__PID_HPP