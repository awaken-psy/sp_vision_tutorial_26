#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/target.hpp"  // 新增：目标跟踪类（包含EKF）
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/exiter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  // 新增：目标跟踪器（使用EKF估计角速度）
  std::unique_ptr<auto_aim::Target> target_tracker = nullptr;
  
  // 新增：射击控制变量
  enum class RotationSpeed { LOW, MEDIUM, HIGH, COMPLETED };
  RotationSpeed current_speed = RotationSpeed::LOW;
  int shot_count = 0;                    // 当前转速下已发射弹丸数量
  const int MAX_SHOTS_PER_SPEED = 10;    // 每个转速最大发射数量
  bool is_shooting = false;              // 是否正在射击
  std::chrono::steady_clock::time_point last_shot_time;
  
  // 新增：弹道补偿相关
  double bullet_speed = 15.0;            // 子弹初速度（m/s）
  const double GRAVITY = 9.8;            // 重力加速度
  const double TARGET_DISTANCE = 2.0;    // 目标距离2m
  
  // 新增：云台瞄准策略 - 瞄准靶机中心
  bool aim_at_center = true;             // 是否瞄准靶机中心
  Eigen::Vector3d target_center;         // 估计的靶机中心位置

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    // Your code start
    
    // ==================== 第一步：数据获取 ====================
    // 1.1 从相机获取图像数据
    camera.read(img, t);
    
    // 1.2 从C板获取当前云台姿态四元数
    q = gimbal.q(t);
    
    // ==================== 第二步：目标检测 ====================
    auto armors = yolo.detect(img);
    
    // ==================== 第三步：目标跟踪与EKF角速度估计 ====================
    if (!armors.empty()) {
      // 3.1 选择最佳目标（这里简单选择第一个检测到的装甲板）
      auto best_armor = armors.front();
      
      // 3.2 设置云台姿态到解算器
      solver.set_R_gimbal2world(q);
      
      // 3.3 解算目标3D位置
      solver.solve(best_armor);
      
      // 3.4 初始化或更新EKF目标跟踪器
      if (target_tracker == nullptr) {
        // 初始化EKF跟踪器
        // 状态向量通常包含位置、速度、角速度等
        Eigen::VectorXd initial_state(9); // [x, y, z, vx, vy, vz, omega, radius, phase]
        initial_state << best_armor.xyz_in_gimbal[0], best_armor.xyz_in_gimbal[1], 
                        best_armor.xyz_in_gimbal[2], 0, 0, 0, 0, 0.2, 0;
        Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Identity(9, 9) * 0.1;
        target_tracker = std::make_unique<auto_aim::Target>(best_armor, t, initial_cov);
      } else {
        // 更新EKF跟踪器
        target_tracker->predict(t);
        target_tracker->update(best_armor);
      }
      
      // 3.5 获取EKF估计的状态
      auto ekf_state = target_tracker->ekf_x();
      
      // 3.6 提取估计的角速度（状态向量中的第7个元素）
      double estimated_omega = ekf_state[6]; // 角速度估计值（rad/s）
      
      // 3.7 估计靶机中心位置（假设装甲板围绕中心旋转）
      // 这里简化处理：使用第一个装甲板位置作为中心估计
      // 实际应用中需要根据多个装甲板位置估计中心
      target_center = ekf_state.head<3>();
      
      // ==================== 第四步：射击控制策略 ====================
      // 4.1 检查是否完成所有转速测试
      if (current_speed == RotationSpeed::COMPLETED) {
        gimbal.send(false, false, 0, 0);
      }
      // 4.2 检查当前转速是否已完成射击
      else if (shot_count >= MAX_SHOTS_PER_SPEED) {
        // 当前转速射击完成，切换到下一转速
        switch (current_speed) {
          case RotationSpeed::LOW:
            current_speed = RotationSpeed::MEDIUM;
            break;
          case RotationSpeed::MEDIUM:
            current_speed = RotationSpeed::HIGH;
            break;
          case RotationSpeed::HIGH:
            current_speed = RotationSpeed::COMPLETED;
            break;
          default:
            break;
        }
        shot_count = 0;
        target_tracker.reset(); // 重置跟踪器
        gimbal.send(false, false, 0, 0);
      }
      // 4.3 正常射击控制
      else {
        // 计算瞄准位置
        Eigen::Vector3d aim_position;
        
        if (aim_at_center) {
          // 策略：瞄准靶机中心位置
          aim_position = target_center;
        } else {
          // 策略：瞄准当前检测到的装甲板
          aim_position = ekf_state.head<3>();
        }
        
        // 计算子弹飞行时间
        double distance = aim_position.norm();
        double flight_time = distance / bullet_speed;
        
        // 预测目标未来位置（考虑旋转）
        // 这里简化处理：假设目标匀速直线运动
        // 实际应用中需要根据角速度预测旋转位置
        Eigen::Vector3d predicted_position = aim_position;
        if (estimated_omega > 0.1) { // 只有角速度较大时才进行预测
          // 简化的位置预测：假设目标在旋转平面内运动
          double prediction_angle = estimated_omega * flight_time;
          // 这里需要根据实际旋转模型进行更精确的预测
          // 简化处理：在垂直方向上添加小的偏移
          predicted_position[1] += 0.1 * sin(prediction_angle);
        }
        
        // 计算弹道补偿
        double drop_distance = 0.5 * GRAVITY * flight_time * flight_time;
        double compensation_angle = atan2(drop_distance, distance);
        
        // 计算瞄准角度
        double raw_yaw = atan2(predicted_position.y(), predicted_position.x());
        double raw_pitch = atan2(-predicted_position.z(), 
                                sqrt(predicted_position.x()*predicted_position.x() + 
                                    predicted_position.y()*predicted_position.y()));
        
        // 应用弹道补偿
        double compensated_pitch = raw_pitch + compensation_angle;
        
        // 角度限制
        raw_yaw = tools::limit_rad(raw_yaw);
        compensated_pitch = tools::limit_min_max(compensated_pitch, -0.35, 0.35);
        
        // 射击决策
        bool should_fire = false;
        
        // 策略1：当云台已经指向目标时射击
        double angle_error = sqrt(raw_yaw * raw_yaw + raw_pitch * raw_pitch);
        if (angle_error < 0.02) { // 误差小于0.02弧度时射击
          should_fire = true;
        }
        
        // 策略2：根据旋转相位射击（简化版）
        // 这里可以根据角速度估计射击时机
        auto current_time = std::chrono::steady_clock::now();
        bool can_fire = !is_shooting || 
                       std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_shot_time).count() > 200;
        
        if (should_fire && can_fire) {
          gimbal.send(true, true, raw_yaw, compensated_pitch);
          shot_count++;
          last_shot_time = current_time;
          is_shooting = true;
        } else {
          gimbal.send(true, false, raw_yaw, compensated_pitch);
          is_shooting = false;
        }
      }
      
      // ==================== 第五步：数据记录和可视化 ====================
      nlohmann::json plot_data;
      plot_data["current_speed"] = static_cast<int>(current_speed);
      plot_data["shot_count"] = shot_count;
      plot_data["estimated_omega"] = estimated_omega;  // EKF估计的角速度
      plot_data["target_yaw"] = 0;  // 瞄准中心，yaw为0
      plot_data["target_pitch"] = 0; // 瞄准中心，pitch为0
      plot_data["is_shooting"] = is_shooting;
      plot_data["bullet_speed"] = bullet_speed;
      
      // 添加角速度误差评估
      double expected_omega = 0;
      double tolerance = 0;
      switch (current_speed) {
        case RotationSpeed::LOW:
          expected_omega = 4.0;  // 低转速中间值
          tolerance = 0.6;
          break;
        case RotationSpeed::MEDIUM:
          expected_omega = 6.0;  // 中转速中间值
          tolerance = 1.0;
          break;
        case RotationSpeed::HIGH:
          expected_omega = 9.0;  // 高转速中间值
          tolerance = 1.5;
          break;
        default:
          break;
      }
      
      double omega_error = std::abs(estimated_omega - expected_omega);
      bool omega_within_tolerance = omega_error <= tolerance;
      
      plot_data["expected_omega"] = expected_omega;
      plot_data["omega_error"] = omega_error;
      plot_data["omega_within_tolerance"] = omega_within_tolerance;
      
      plotter.plot(plot_data);
      
      // 调试显示
      std::string speed_str;
      switch (current_speed) {
        case RotationSpeed::LOW: speed_str = "LOW"; break;
        case RotationSpeed::MEDIUM: speed_str = "MEDIUM"; break;
        case RotationSpeed::HIGH: speed_str = "HIGH"; break;
        case RotationSpeed::COMPLETED: speed_str = "COMPLETED"; break;
      }
      
      std::string info = "Speed: " + speed_str + 
                        " Shots: " + std::to_string(shot_count) + 
                        "/" + std::to_string(MAX_SHOTS_PER_SPEED);
      tools::draw_text(img, info, cv::Point(10, 30), cv::Scalar(0, 255, 255));
      
      std::string omega_info = "Omega: " + std::to_string(estimated_omega) + 
                              " rad/s (Est)";
      tools::draw_text(img, omega_info, cv::Point(10, 60), cv::Scalar(0, 255, 255));
      
      std::string tolerance_info = "Tolerance: ±" + std::to_string(tolerance) + 
                                  " rad/s";
      tools::draw_text(img, tolerance_info, cv::Point(10, 90), cv::Scalar(0, 255, 255));
      
      if (omega_within_tolerance) {
        tools::draw_text(img, "WITHIN TOLERANCE", cv::Point(10, 120), cv::Scalar(0, 255, 0));
      } else {
        tools::draw_text(img, "OUT OF TOLERANCE", cv::Point(10, 120), cv::Scalar(0, 0, 255));
      }
      
    } else {
      // 没有检测到目标
      gimbal.send(false, false, 0, 0);
      
      nlohmann::json plot_data;
      plot_data["armor_detected"] = false;
      plot_data["current_speed"] = static_cast<int>(current_speed);
      plotter.plot(plot_data);
      
      tools::draw_text(img, "No Target", cv::Point(10, 30), cv::Scalar(0, 0, 255));
    }
    
    // 显示图像
    cv::imshow("Auto Aim - Task 3 (Spinning Target)", img);
    cv::waitKey(1);
    
    // Your code end
  }

  return 0;
}