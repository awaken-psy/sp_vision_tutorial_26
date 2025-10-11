#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"
#include <iostream>
#include <filesystem>

int main()
{
    // 初始化相机
    myCamera camera;
    
    // 检查配置文件路径
    std::string config_path = "./configs/yolo.yaml";
    if (!std::filesystem::exists(config_path)) {
        std::cerr << "Config file not found: " << config_path << std::endl;
        return -1;
    }
    
    try {
        // 初始化yolo类，使用配置文件路径
        auto_aim::YOLO yolo_detector(config_path, true);  // 第二个参数是debug模式
        
        std::cout << "YOLO detector initialized successfully!" << std::endl;
        
        int frame_count = 0;
        
        while (true) {
            // 调用相机读取图像
            cv::Mat img = camera.read();
            if (img.empty()) {
                std::cerr << "Failed to read image from camera!" << std::endl;
                
                // 创建测试图像（如果没有相机）
                img = cv::Mat::zeros(480, 640, CV_8UC3);
                cv::putText(img, "No Camera - Press 'q' to quit", 
                           cv::Point(50, 240), cv::FONT_HERSHEY_SIMPLEX, 
                           0.7, cv::Scalar(255, 255, 255), 2);
            }
            
            // 调用yolo识别装甲板
            std::list<auto_aim::Armor> armors = yolo_detector.detect(img, frame_count);
            
            // 在图像上绘制检测结果
            for (const auto& armor : armors) {
                // 使用红色绘制装甲板的四个关键点（闭合矩形）
                tools::draw_points(img, armor.points, cv::Scalar(0, 0, 255), 2);
                
                // 在装甲板中心绘制文本信息
                std::string armor_info = auto_aim::COLORS[armor.color] + " " + 
                                       auto_aim::ARMOR_NAMES[armor.name];
                tools::draw_text(img, armor_info, 
                               cv::Point(armor.points[0].x, armor.points[0].y - 10),
                               cv::Scalar(0, 255, 255), 0.5, 2);
            }
            
            // 显示检测到的装甲板数量
            if (!armors.empty()) {
                std::string count_info = "Detected: " + std::to_string(armors.size()) + " armors";
                tools::draw_text(img, count_info, cv::Point(10, 30),
                               cv::Scalar(0, 255, 0), 3, 2);
            }
            
            // 显示图像
            cv::resize(img, img, cv::Size(1280, 960));
            cv::imshow("Armor Detection", img);
            
            frame_count++;
            
            // 按'q'键退出
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error initializing YOLO: " << e.what() << std::endl;
        std::cerr << "Please check:" << std::endl;
        std::cerr << "1. configs/yolo.yaml exists and is valid" << std::endl;
        std::cerr << "2. assets/yolov5.xml and assets/yolov5.bin exist" << std::endl;
        std::cerr << "3. OpenVINO is properly installed" << std::endl;
        return -1;
    }

    return 0;
}