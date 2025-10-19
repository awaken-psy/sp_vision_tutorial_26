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
  double bullet_speed = 15.0;            // 子弹初速度（m/s）//111
  const double GRAVITY = 9.8;            // 重力加速度
  
  // 新增：云台瞄准策略 - 瞄准靶机中心
  bool aim_at_center = true;             // 是否瞄准靶机中心
  Eigen::Vector3d target_center;         // 估计的靶机中心位置

  cv::Mat img;                                          // 存储相机图像
  Eigen::Quaterniond gimbal_quat;                       // 存储云台姿态四元数
  std::chrono::steady_clock::time_point img_timestamp;  // 图像时间戳

  while (!exiter.exit()) {
    // Your code start

    // ==================== 第一步：数据获取 ====================
    // 1.1 从相机获取图像数据
    camera.read(img, img_timestamp);

    // 1.2 从C板获取当前云台姿态四元数
    gimbal_quat = gimbal.q(img_timestamp);

    // ==================== 第二步：目标检测 ====================
    auto armors = yolo.detect(img);

    // ==================== 第三步：目标跟踪与EKF角速度估计 ====================
    if (!armors.empty()) {
      // 3.1 选择最佳目标（这里简单选择第一个检测到的装甲板）
      auto best_armor = armors.front();
      
      // 3.2 设置云台姿态到解算器
      solver.set_R_gimbal2world(gimbal_quat);
      
      // 3.3 解算目标3D位置
      solver.solve(best_armor);
      
      // 3.4 初始化或更新EKF目标跟踪器
      if (target_tracker == nullptr) {
        // 初始化EKF跟踪器
        // 状态向量通常包含位置、速度、角速度等
        Eigen::VectorXd initial_state(9); // [x, y, z, vx, vy, vz, omega, radius, phase(相位)]
        initial_state << best_armor.xyz_in_gimbal[0], best_armor.xyz_in_gimbal[1], 
                        best_armor.xyz_in_gimbal[2], 0, 0, 0, 0, 0.2, 0;//半径对吗111
        Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Identity(9, 9) * 0.1;
        target_tracker = std::make_unique<auto_aim::Target>(best_armor, img_timestamp, initial_cov);
      } else {
        // 更新EKF跟踪器
        target_tracker->predict(img_timestamp);
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

        // ==================== 考虑靶机旋转的预测 ====================
        // 4.3.1 计算子弹飞行时间（初步估算）
        double initial_distance = aim_position.norm();
        double initial_flight_time = initial_distance / bullet_speed;

        // 4.3.2 预测目标在子弹飞行期间的位置变化
        // 假设靶机绕中心旋转，角速度为estimated_omega
        if (std::abs(estimated_omega) > 0.1) {  // 只有角速度较大时才进行预测
          // 计算目标相对于中心的向量
          Eigen::Vector3d relative_pos = aim_position - target_center;

          // 计算当前相位角
          double current_phase = atan2(relative_pos.y(), relative_pos.x());

          // 预测相位角变化
          double phase_change = estimated_omega * initial_flight_time;
          double predicted_phase = current_phase + phase_change;

          // 计算预测后的相对位置（保持相同半径）
          double radius = relative_pos.head<2>().norm();
          Eigen::Vector3d predicted_relative_pos;
          predicted_relative_pos.x() = radius * cos(predicted_phase);
          predicted_relative_pos.y() = radius * sin(predicted_phase);
          predicted_relative_pos.z() = relative_pos.z();  // 高度不变

          // 更新瞄准位置为预测位置
          aim_position = target_center + predicted_relative_pos;

          // 重新计算距离（因为位置变了）
          initial_distance = aim_position.norm();
        }

        // ==================== 使用斜抛模型的精确弹道补偿 ====================
        // 4.3.3 首先确定偏航角（水平方向）
        double yaw = atan2(aim_position.y(), aim_position.x());
        yaw = tools::limit_rad(yaw);

        // 4.3.4 计算在偏航角方向上的水平距离
        double horizontal_distance =
          sqrt(aim_position.x() * aim_position.x() + aim_position.y() * aim_position.y());

        // 4.3.5 获取目标高度（在云台坐标系中，z通常表示高度）
        double target_height = aim_position.z();

        // 4.3.6 建立二维斜抛运动方程求解俯仰角
        // 运动方程：
        // x = v₀ * cos(θ) * t  (水平方向)
        // y = v₀ * sin(θ) * t - 0.5 * g * t²  (垂直方向)
        // 其中：x = 水平距离, y = 目标高度, v₀ = 子弹速度, θ = 俯仰角, t = 飞行时间

        // 从水平运动方程得到：t = x / (v₀ * cos(θ))
        // 代入垂直运动方程：
        // y = v₀ * sin(θ) * (x / (v₀ * cos(θ))) - 0.5 * g * (x / (v₀ * cos(θ)))²
        // 简化得：y = x * tan(θ) - (g * x²) / (2 * v₀² * cos²(θ))

        // 利用三角恒等式：1/cos²(θ) = 1 + tan²(θ)
        // 令 u = tan(θ)，则方程变为：
        // y = x * u - (g * x²) / (2 * v₀²) * (1 + u²)

        // 整理为标准二次方程形式：
        // (g * x²) / (2 * v₀²) * u² - x * u + (g * x²) / (2 * v₀²) + y = 0

        double x = horizontal_distance;
        double y = target_height;
        double v0 = bullet_speed;
        double g = GRAVITY;

        // 计算二次方程系数
        double A = (g * x * x) / (2 * v0 * v0);
        double B = -x;
        double C = A + y;

        // 4.3.7 解二次方程求俯仰角
        double discriminant = B * B - 4 * A * C;
        double pitch;

        if (discriminant >= 0) {
          // 方程有实数解，计算两个可能的俯仰角
          double u1 = (-B + sqrt(discriminant)) / (2 * A);
          double u2 = (-B - sqrt(discriminant)) / (2 * A);

          // 选择较小的俯仰角（更平的弹道，飞行时间更短）
          // 较大的俯仰角对应高抛弹道，飞行时间更长
          double u = (fabs(u1) < fabs(u2)) ? u1 : u2;

          // 将u = tan(θ)转换为俯仰角θ
          pitch = atan(u);

          // 验证解的有效性
          double flight_time = x / (v0 * cos(pitch));
          double calculated_height =
            v0 * sin(pitch) * flight_time - 0.5 * g * flight_time * flight_time;

          // 如果计算高度与实际高度差异过大，使用另一个解
          if (fabs(calculated_height - y) > 0.1) {
            u = (u == u1) ? u2 : u1;
            pitch = atan(u);
          }
        } else {
          // 无实数解，目标在射程外，使用直线瞄准作为备选
          pitch = atan2(y, x);
          tools::logger()->warn("Target out of range, using straight line aim");
        }

        // 4.3.8 角度限制
        pitch = tools::limit_min_max(pitch, -0.35, 0.35);

        // 4.3.9 计算弹道参数
        double flight_time = horizontal_distance / (bullet_speed * cos(pitch));
        double max_height = (bullet_speed * sin(pitch) * bullet_speed * sin(pitch)) / (2 * GRAVITY);

        // ==================== 射击决策 ====================
        // 计算角度误差，判断是否瞄准目标
        double angle_error = sqrt(yaw * yaw + pitch * pitch);

        // 如果瞄准误差足够小，进行射击
        bool should_fire = (angle_error < 0.02);  // 误差小于0.02弧度时射击

        // 射击频率控制（避免连续快速射击）
        auto current_time = std::chrono::steady_clock::now();
        bool can_fire = !is_shooting || std::chrono::duration_cast<std::chrono::milliseconds>(
                                          current_time - last_shot_time)
                                            .count() > 200;

        if (should_fire && can_fire) {
          gimbal.send(true, true, yaw, pitch);
          shot_count++;
          last_shot_time = current_time;
          is_shooting = true;
        } else {
          gimbal.send(true, false, yaw, pitch);
          is_shooting = false;
        }

        // ==================== 第五步：数据记录和可视化 ====================
        nlohmann::json plot_data;
        plot_data["target_yaw"] = yaw;
        plot_data["target_pitch"] = pitch;
        plot_data["compensation_angle"] = pitch - atan2(target_height, horizontal_distance);  // 补偿角度
        plot_data["target_distance"] = sqrt(x * x + y * y);
        plot_data["flight_time"] = flight_time;
        plot_data["max_height"] = max_height;
        plot_data["rotation_prediction"] = (std::abs(estimated_omega) > 0.1);  // 是否进行了旋转预测
        
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