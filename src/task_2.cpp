#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"  // 新增：目标跟踪类
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

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
  int shot_count = 0;                                    // 当前位置已发射弹丸数量
  int position_count = 0;                                // 已完成的射击位置数量
  const int MAX_SHOTS_PER_POSITION = 10;                 // 每个位置最大发射数量
  const int TOTAL_POSITIONS = 3;                         // 总射击位置数量
  bool is_shooting = false;                              // 是否正在射击
  std::chrono::steady_clock::time_point last_shot_time;  // 上次射击时间

  // 新增：弹道补偿相关
  double bullet_speed = 15.0;  // 子弹初速度（m/s），需要根据实际情况调整//111
  const double GRAVITY = 9.8;  // 重力加速度

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

    // ==================== 第三步：目标跟踪与滤波 ====================
    if (!armors.empty()) {
      // 3.1 选择最佳目标（这里简单选择第一个检测到的装甲板）
      auto best_armor = armors.front();

      // 3.2 设置云台姿态到解算器
      solver.set_R_gimbal2world(gimbal_quat);

      // 3.3 解算目标3D位置
      solver.solve(best_armor);

      // 3.4 初始化或更新卡尔曼滤波器
      if (target_tracker == nullptr) {
        // 第一次检测到目标，初始化卡尔曼滤波器
        // NOTES: 使用的是gimbal的xyz坐标系
        Eigen::VectorXd initial_state(6);  // 假设状态向量为[x, y, z, vx, vy, vz]
        initial_state << best_armor.xyz_in_gimbal[0], best_armor.xyz_in_gimbal[1],
          best_armor.xyz_in_gimbal[2], 0, 0, 0;
        Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        target_tracker = std::make_unique<auto_aim::Target>(best_armor, img_timestamp, initial_cov);
      } else {
        // 更新卡尔曼滤波器
        target_tracker->predict(img_timestamp);
        target_tracker->update(best_armor);
      }

      // 3.5 获取滤波后的目标状态
      auto filtered_state = target_tracker->ekf_x();
      Eigen::Vector3d filtered_position = filtered_state.head<3>();

      // ==================== 精确弹道补偿计算 ====================
      // 使用二维斜抛运动模型，考虑重力影响的精确弹道计算

      // 4.1 首先确定偏航角（水平方向）
      double yaw = atan2(filtered_position.y(), filtered_position.x());
      yaw = tools::limit_rad(yaw);

      // 4.2 计算在偏航角方向上的水平距离
      // 将3D位置投影到偏航角方向的2D平面上
      double horizontal_distance = sqrt(
        filtered_position.x() * filtered_position.x() +
        filtered_position.y() * filtered_position.y());

      // 4.3 获取目标高度（在云台坐标系中，z通常表示高度）
      double target_height = filtered_position.z();

      // 4.4 建立二维斜抛运动方程求解俯仰角
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

      // 4.5 解二次方程求俯仰角
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

      // 4.6 角度限制
      pitch = tools::limit_min_max(pitch, -0.35, 0.35);

      // 4.7 可选：计算并记录弹道参数用于调试
      double flight_time = horizontal_distance / (bullet_speed * cos(pitch));
      double max_height = (bullet_speed * sin(pitch) * bullet_speed * sin(pitch)) / (2 * GRAVITY);

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
        if (
          std::chrono::duration_cast<std::chrono::seconds>(current_time - last_shot_time).count() >
          2) {
          shot_count = 0;
          position_count++;
          target_tracker.reset();  // 重置跟踪器，重新初始化
        }
        gimbal.send(false, false, 0, 0);
      }
      // 5.3 正常射击控制
      else {
        // 计算角度误差，判断是否瞄准目标
        double angle_error = sqrt(yaw * yaw + pitch * pitch);

        // 如果瞄准误差足够小，进行射击
        bool should_fire = (angle_error < 0.05);  // 误差小于0.05弧度时射击

        // 射击频率控制（避免连续快速射击）
        auto current_time = std::chrono::steady_clock::now();
        bool can_fire = !is_shooting || std::chrono::duration_cast<std::chrono::milliseconds>(
                                          current_time - last_shot_time)
                                            .count() > 200;

        if (should_fire && can_fire && shot_count < MAX_SHOTS_PER_POSITION) {
          gimbal.send(true, true, yaw, pitch);
          shot_count++;
          last_shot_time = current_time;
          is_shooting = true;
        } else {
          gimbal.send(true, false, yaw, pitch);
          is_shooting = false;
        }
      }

      // ==================== 第六步：数据记录和可视化 ====================
      nlohmann::json plot_data;
      plot_data["position_count"] = position_count;
      plot_data["shot_count"] = shot_count;
      plot_data["target_yaw"] = yaw;
      plot_data["target_pitch"] = pitch;
      plot_data["is_shooting"] = is_shooting;
      plot_data["bullet_speed"] = bullet_speed;
      plotter.plot(plot_data);

    } else {
      // 没有检测到目标
      gimbal.send(false, false, 0, 0);
      target_tracker.reset();  // 重置跟踪器

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