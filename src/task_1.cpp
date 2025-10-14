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
  io::Camera camera(config_path);  // 相机：获取图像数据
  io::Gimbal gimbal(config_path);  // 云台：获取姿态数据 + 发送控制指令

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);  // 目标检测
  auto_aim::Solver solver(config_path);    // 坐标解算

  // 变量定义
  cv::Mat img;                              // 存储相机图像
  std::chrono::steady_clock::time_point img_timestamp;  // 图像时间戳
  Eigen::Quaterniond gimbal_quat;           // 存储云台姿态四元数
  std::chrono::steady_clock::time_point quat_timestamp; // 四元数时间戳

  while (!exiter.exit()) {
    // Your code start
    
    // ==================== 第一步：数据获取 ====================
    // 1.1 从相机获取图像数据
    camera.read(img, img_timestamp);
    
    // 1.2 从C板（云台主控）获取当前姿态四元数
    // 注意：这里需要确保四元数的时间戳与图像时间戳同步或插值
    gimbal_quat = gimbal.q(img_timestamp);  // 获取对应时间点的云台姿态
    
    // ==================== 第二步：目标检测 ====================
    // 2.1 使用YOLO检测图像中的装甲板
    auto armors = yolo.detect(img);
    
    // 2.2 检查是否检测到装甲板
    if (!armors.empty()) {
      // 选择第一个检测到的装甲板作为目标
      auto target_armor = armors.front();
      
      // ==================== 第三步：坐标解算 ====================
      // 3.1 设置当前云台姿态到解算器
      solver.set_R_gimbal2world(gimbal_quat);
      
      // 3.2 解算装甲板在3D空间中的位置
      solver.solve(target_armor);
      // 解算完成后，target_armor中将包含：
      // - xyz_in_gimbal: 在云台坐标系中的3D坐标
      // - ypr_in_gimbal: 在云台坐标系中的偏航俯仰角
      
      // ==================== 第四步：控制指令生成 ====================
      // 4.1 从解算结果中获取目标相对于云台的偏航和俯仰角
      double yaw = target_armor.ypr_in_gimbal[0];    // 偏航角（弧度）
      double pitch = target_armor.ypr_in_gimbal[1];  // 俯仰角（弧度）
      
      // 4.2 角度限制（确保在云台运动范围内）
      // yaw: 通常不需要限制，云台可以360°旋转
      // pitch: 限制在±20°（约±0.35弧度）范围内
      pitch = tools::limit_min_max(pitch, -0.35, 0.35);
      
      // 4.3 发送控制指令到C板
      // 参数说明：
      // - true: 启用云台控制
      // - false: 不射击
      // - yaw: 目标偏航角
      // - pitch: 目标俯仰角
      gimbal.send(true, false, yaw, pitch);
      
      // ==================== 第五步：数据记录和可视化 ====================
      // 5.1 发送数据到Plotter用于考核记录
      nlohmann::json plot_data;
      plot_data["control_enable"] = true;
      plot_data["fire_enable"] = false;
      plot_data["target_yaw"] = yaw;
      plot_data["target_pitch"] = pitch;
      plot_data["armor_detected"] = true;
      plot_data["armor_center_x"] = target_armor.center.x;
      plot_data["armor_center_y"] = target_armor.center.y;
      plotter.plot(plot_data);
      
      // 5.2 在图像上绘制检测结果（调试用）
      // 绘制装甲板中心点
      tools::draw_point(img, target_armor.center, cv::Scalar(0, 255, 0), 5);
      // 显示控制信息
      std::string yaw_str = "Yaw: " + std::to_string(yaw * 180 / M_PI) + "°";
      std::string pitch_str = "Pitch: " + std::to_string(pitch * 180 / M_PI) + "°";
      tools::draw_text(img, yaw_str, cv::Point(10, 30), cv::Scalar(0, 255, 255));
      tools::draw_text(img, pitch_str, cv::Point(10, 60), cv::Scalar(0, 255, 255));
      
    } else {
      // ==================== 无目标处理 ====================
      // 没有检测到装甲板时，停止云台控制
      gimbal.send(false, false, 0, 0);
      
      // 发送无目标状态到Plotter
      nlohmann::json plot_data;
      plot_data["control_enable"] = false;
      plot_data["armor_detected"] = false;
      plotter.plot(plot_data);
      
      // 显示无目标信息
      tools::draw_text(img, "No Armor Detected", cv::Point(10, 30), cv::Scalar(0, 0, 255));
    }
    
    // ==================== 显示图像 ====================
    cv::imshow("Auto Aim - Task 1", img);
    cv::waitKey(1);
    
    // Your code end
  }

  return 0;
}