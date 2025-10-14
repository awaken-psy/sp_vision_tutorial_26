// 防止头文件重复包含
#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>  // Eigen几何变换库
#include <chrono>          // 时间库

namespace tools
{
// 将弧度值限制在(-pi, pi]范围内（处理角度环绕）
double limit_rad(double angle);

// 四元数转欧拉角
// x = 0, y = 1, z = 2
// 例如：先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
// extrinsic: 是否为外旋（固定坐标系），false表示内旋（随体坐标系）
// 参考：https://github.com/evbernardes/quaternion_to_euler
Eigen::Vector3d eulers(
  Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);

// 旋转矩阵转欧拉角
// x = 0, y = 1, z = 2
// 例如：先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
// extrinsic: 是否为外旋（固定坐标系）
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

// 欧拉角转旋转矩阵
// zyx顺序：先绕z轴旋转（偏航），再绕y轴旋转（俯仰），最后绕x轴旋转（横滚）
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr);

// 直角坐标系转球坐标系
// ypd为yaw（偏航角）、pitch（俯仰角）、distance（距离）的缩写
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

// 直角坐标系转球坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

// 球坐标系转直角坐标系
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd);

// 球坐标系转直角坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd);

// 计算时间差a - b，单位：秒
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

// 计算两个向量之间的夹角，返回0 ~ pi范围内的绝对值（来自SJTU算法）
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2);

// 模板函数：返回输入值的平方
template <typename T>
T square(T const & a)
{
  return a * a;
};

// 将输入值限制在[min, max]范围内
double limit_min_max(double input, double min, double max);
}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP