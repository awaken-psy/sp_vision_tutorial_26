#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/exiter.hpp"
#include "tasks/auto_aim/target.hpp"  // 新增：目标跟踪类

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

  // 新增：目标跟踪器（卡尔曼滤波）
  std::unique_ptr<auto_aim::Target> target_tracker = nullptr;
  
  // 新增：射击控制变量
  int shot_count = 0;                    // 当前位置已发射弹丸数量
  int position_count = 0;                // 已完成的射击位置数量
  const int MAX_SHOTS_PER_POSITION = 10; // 每个位置最大发射数量
  const int TOTAL_POSITIONS = 3;         // 总射击位置数量
  bool is_shooting = false;              // 是否正在射击
  std::chrono::steady_clock::time_point last_shot_time; // 上次射击时间
  
  // 新增：弹道补偿相关
  double bullet_speed = 15.0;            // 子弹初速度（m/s），需要根据实际情况调整
  const double GRAVITY = 9.8;            // 重力加速度

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
    
    // ==================== 第三步：目标跟踪与滤波 ====================
    if (!armors.empty()) {
      // 3.1 选择最佳目标（这里简单选择第一个检测到的装甲板）
      auto best_armor = armors.front();
      
      // 3.2 设置云台姿态到解算器
      solver.set_R_gimbal2world(q);
      
      // 3.3 解算目标3D位置
      solver.solve(best_armor);
      
      // 3.4 初始化或更新卡尔曼滤波器
      if (target_tracker == nullptr) {
        // 第一次检测到目标，初始化卡尔曼滤波器
        Eigen::VectorXd initial_state(6); // 假设状态向量为[x, y, z, vx, vy, vz]
        initial_state << best_armor.xyz_in_gimbal[0], best_armor.xyz_in_gimbal[1], 
                        best_armor.xyz_in_gimbal[2], 0, 0, 0;
        Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        target_tracker = std::make_unique<auto_aim::Target>(best_armor, t, initial_cov);
      } else {
        // 更新卡尔曼滤波器
        target_tracker->predict(t);
        target_tracker->update(best_armor);
      }
      
      // 3.5 获取滤波后的目标状态
      auto filtered_state = target_tracker->ekf_x();
      Eigen::Vector3d filtered_position = filtered_state.head<3>();
      
      // ==================== 第四步：弹道补偿 ====================
      // 4.1 计算目标距离
      double distance = filtered_position.norm();
      
      // 4.2 计算子弹飞行时间
      double flight_time = distance / bullet_speed;
      
      // 4.3 计算重力引起的下坠距离
      double drop_distance = 0.5 * GRAVITY * flight_time * flight_time;
      
      // 4.4 计算弹道补偿角度
      double compensation_angle = atan2(drop_distance, distance);
      
      // 4.5 计算原始指向角度
      double raw_yaw = atan2(filtered_position.y(), filtered_position.x());
      double raw_pitch = atan2(-filtered_position.z(), 
                              sqrt(filtered_position.x()*filtered_position.x() + 
                                  filtered_position.y()*filtered_position.y()));
      
      // 4.6 应用弹道补偿（俯仰角向上补偿）
      double compensated_pitch = raw_pitch + compensation_angle;
      
      // 角度限制
      raw_yaw = tools::limit_rad(raw_yaw);
      compensated_pitch = tools::limit_min_max(compensated_pitch, -0.35, 0.35);
      
      // ==================== 第五步：射击控制 ====================
      // 5.1 检查是否完成所有位置射击
      if (position_count >= TOTAL_POSITIONS) {
        // 所有位置射击完成，停止控制
        gimbal.send(false, false, 0, 0);
      }
      // 5.2 检查当前位置是否已完成射击
      else if (shot_count >= MAX_SHOTS_PER_POSITION) {
        // 当前位置射击完成，等待目标移动到下一位置
        // 这里可以通过目标位置变化来判断是否移动到新位置
        // 简化处理：等待一段时间后重置计数器
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_shot_time).count() > 2) {
          shot_count = 0;
          position_count++;
          target_tracker.reset(); // 重置跟踪器，重新初始化
        }
        gimbal.send(false, false, 0, 0);
      }
      // 5.3 正常射击控制
      else {
        // 计算角度误差，判断是否瞄准目标
        double angle_error = sqrt(raw_yaw * raw_yaw + raw_pitch * raw_pitch);
        
        // 如果瞄准误差足够小，进行射击
        bool should_fire = (angle_error < 0.05); // 误差小于0.05弧度时射击
        
        // 射击频率控制（避免连续快速射击）
        auto current_time = std::chrono::steady_clock::now();
        bool can_fire = !is_shooting || 
                       std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_shot_time).count() > 200;
        
        if (should_fire && can_fire && shot_count < MAX_SHOTS_PER_POSITION) {
          gimbal.send(true, true, raw_yaw, compensated_pitch);
          shot_count++;
          last_shot_time = current_time;
          is_shooting = true;
        } else {
          gimbal.send(true, false, raw_yaw, compensated_pitch);
          is_shooting = false;
        }
      }
      
      // ==================== 第六步：数据记录和可视化 ====================
      nlohmann::json plot_data;
      plot_data["position_count"] = position_count;
      plot_data["shot_count"] = shot_count;
      plot_data["target_yaw"] = raw_yaw;
      plot_data["target_pitch"] = compensated_pitch;
      plot_data["compensation_angle"] = compensation_angle;
      plot_data["target_distance"] = distance;
      plot_data["is_shooting"] = is_shooting;
      plot_data["bullet_speed"] = bullet_speed;
      plotter.plot(plot_data);
      
      // 调试显示
      std::string info = "Pos: " + std::to_string(position_count) + 
                        " Shots: " + std::to_string(shot_count) + 
                        "/" + std::to_string(MAX_SHOTS_PER_POSITION);
      tools::draw_text(img, info, cv::Point(10, 30), cv::Scalar(0, 255, 255));
      
      std::string angle_info = "Yaw: " + std::to_string(raw_yaw * 180 / M_PI) + 
                              "° Pitch: " + std::to_string(compensated_pitch * 180 / M_PI) + "°";
      tools::draw_text(img, angle_info, cv::Point(10, 60), cv::Scalar(0, 255, 255));
      
      std::string comp_info = "Comp: " + std::to_string(compensation_angle * 180 / M_PI) + 
                             "° Dist: " + std::to_string(distance) + "m";
      tools::draw_text(img, comp_info, cv::Point(10, 90), cv::Scalar(0, 255, 255));
      
    } else {
      // 没有检测到目标
      gimbal.send(false, false, 0, 0);
      target_tracker.reset(); // 重置跟踪器
      
      nlohmann::json plot_data;
      plot_data["armor_detected"] = false;
      plotter.plot(plot_data);
      
      tools::draw_text(img, "No Target", cv::Point(10, 30), cv::Scalar(0, 0, 255));
    }
    
    // 显示图像
    cv::imshow("Auto Aim - Task 2", img);
    cv::waitKey(1);
    
    // Your code end
  }

  return 0;
}