// 防止头文件重复包含
#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

// Eigen库必须在opencv2/core/eigen.hpp上面包含，避免编译错误
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>  // OpenCV与Eigen的转换头文件

#include "armor.hpp"  // 装甲板数据结构

namespace auto_aim
{
// 求解器类：负责坐标变换、位姿解算和重投影等几何计算
class Solver
{
public:
  // 显式构造函数：通过配置文件路径初始化
  explicit Solver(const std::string & config_path);

  // 获取从云台坐标系到世界坐标系的旋转矩阵
  Eigen::Matrix3d R_gimbal2world() const;

  // 设置从云台坐标系到世界坐标系的旋转矩阵（通过四元数）
  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  // 主要求解函数：计算装甲板在三维空间中的位置和姿态
  void solve(Armor & armor) const;

  // 重投影函数：将世界坐标系中的3D装甲板投影到图像平面
  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  // 计算前哨站装甲板的重投影误差
  double oupost_reprojection_error(Armor armor, const double & picth);

  // 将世界坐标系中的3D点转换到图像像素坐标系
  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  // 相机内参矩阵和畸变系数
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  
  // 坐标系变换矩阵
  Eigen::Matrix3d R_gimbal2imubody_;  // 云台到IMU体的旋转矩阵
  Eigen::Matrix3d R_camera2gimbal_;   // 相机到云台的旋转矩阵
  Eigen::Vector3d t_camera2gimbal_;   // 相机到云台的平移向量
  Eigen::Matrix3d R_gimbal2world_;    // 云台到世界的旋转矩阵

  // 优化装甲板的偏航角
  void optimize_yaw(Armor & armor) const;

  // 计算装甲板的重投影误差
  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  
  // 重投影代价函数
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP