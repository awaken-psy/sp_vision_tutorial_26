// 包含必要的头文件
#include "tasks/buff_detector.hpp" // 自定义的能量机关检测器
#include "tasks/buff_solver.hpp"   // 自定义的能量机关求解器
#include <chrono>                  // 时间库
#include <nlohmann/json.hpp>       // JSON库，用于数据序列化
#include <opencv2/opencv.hpp>      // OpenCV计算机视觉库
#include "tools/plotter.hpp"       // 自定义绘图工具，用于数据可视化

//  相机内参
static const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) << 1286.307063384126, 0, 645.34450819155256,
     0, 1288.1400736562441, 483.6163720308021,
     0, 0, 1);
// 畸变系数
static const cv::Mat distort_coeffs =
    (cv::Mat_<double>(1, 5) << -0.47562935060124745, 0.21831745829617311, 0.0004957613589406044, -0.00034617769548693592, 0);

int main()
{
    // 1. 初始化视频捕获对象，打开测试视频文件
    cv::VideoCapture cap("assets/test.avi");

    // 检查视频文件是否成功打开
    if (!cap.isOpened())
    {
        std::cerr << "无法打开视频文件!" << std::endl;
        return -1;
    }

    // 2. 初始化检测器和绘图器
    auto_buff::Buff_Detector detector; // 创建能量机关检测器实例
    auto_buff::Buff_Solver solver;     // 创建能量机关求解器实例
    tools::Plotter plotter;            // 创建数据绘图器实例，用于实时数据可视化

    // 3. 主循环：逐帧处理视频
    while (true)
    {
        cv::Mat img; // 存储当前帧图像
        cap >> img;  // 从视频中读取一帧

        // 检查是否成功读取帧（视频结束或读取失败）
        if (img.empty())
        {
            std::cout << "视频播放完毕或无法读取帧" << std::endl;
            break; // 退出循环
        }

        // 4. 使用检测器检测当前帧中的扇叶目标
        auto fanblades = detector.detect(img);

        // 5. 创建显示图像的副本，用于绘制检测结果
        cv::Mat display_img = img.clone();
        nlohmann::json data; // 创建JSON对象存储数据
    

        // 6. 遍历所有检测到的扇叶，在图像上绘制可视化信息
        int fanblade_count = 0;
        for (const auto &fanblade : fanblades)
        {
            cv::Scalar color;      // 根据扇叶类型设置颜色
            std::string type_name; // 扇叶类型名称

            // 根据扇叶类型设置对应的颜色和名称
            switch (fanblade.type)
            {
            case auto_buff::_target:           // 目标扇叶
                color = cv::Scalar(0, 255, 0); // 绿色
                type_name = "_target";
                break;
            case auto_buff::_light:              // 亮扇叶
                color = cv::Scalar(0, 255, 255); // 黄色
                type_name = "_light";
                break;
            case auto_buff::_unlight:          // 未亮扇叶
                color = cv::Scalar(0, 0, 255); // 红色
                type_name = "_unlight";
                break;
            }

            // 绘制关键点：在扇叶的各个特征点上画圆并标注序号
            for (size_t i = 0; i < fanblade.points.size(); ++i)
            {
                cv::circle(display_img, fanblade.points[i], 3, color, -1); // 画实心圆点
                cv::putText(display_img, std::to_string(i),                // 标注点序号
                            cv::Point(fanblade.points[i].x + 5, fanblade.points[i].y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }

            // 绘制中心点：在扇叶中心画圆并标注
            cv::circle(display_img, fanblade.center, 5, color, -1); // 画中心点
            cv::putText(display_img, "CENTER",                      // 标注"中心"
                        cv::Point(fanblade.center.x + 10, fanblade.center.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);

            // 绘制类型标签：在中心点附近显示扇叶类型
            cv::putText(display_img, type_name,
                        cv::Point(fanblade.center.x - 20, fanblade.center.y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

            // 进行PnP解算
            auto_buff::Buff_Solver::Solution solution;
            solver.solvePnP(fanblade.points, camera_matrix, distort_coeffs, solution);

            if (solution.valid)
            {
                // 绘制解算得到的符中心
                cv::circle(display_img, solution.fan_center, 8, cv::Scalar(255, 0, 0), -1);
                cv::putText(display_img, "FAN_CENTER",
                            cv::Point(solution.fan_center.x + 10, solution.fan_center.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

                // 绘制解算得到的旋转中心
                cv::circle(display_img, solution.rotation_center, 8, cv::Scalar(0, 0, 255), -1);
                cv::putText(display_img, "ROTATION_CENTER",
                            cv::Point(solution.rotation_center.x + 10, solution.rotation_center.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

                // 绘制从符中心到旋转中心的连线
                cv::line(display_img, solution.fan_center, solution.rotation_center,
                         cv::Scalar(255, 255, 0), 2);

                // 发送数据到plotjuggler
                if(fanblade_count == 0) { // 只记录第一个扇叶的数据
                data["fan_center_x"] = solution.fan_center.x;
                data["fan_center_y"] = solution.fan_center.y;
                data["rotation_center_x"] = solution.rotation_center.x;
                data["rotation_center_y"] = solution.rotation_center.y;
                }
            }
            
            fanblade_count++;
        }

        // 7. 显示检测结果窗口
        cv::resize(display_img, display_img, {}, 0.8, 0.8); // 缩放图像到80%大小
        cv::imshow("Detection Results", display_img);       // 显示处理后的图像
        plotter.plot(data);
        

        if (cv::waitKey(30) == 27)
        {
            break;
        }
    }

    // 10. 资源清理
    cap.release();           // 释放视频捕获资源
    cv::destroyAllWindows(); // 关闭所有OpenCV窗口

    return 0; // 程序正常退出
}