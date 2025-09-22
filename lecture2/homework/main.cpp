#include "tools.hpp"
#include <opencv2/opencv.hpp>
#include <fmt/core.h>
#include <iostream>

int main(int argc, char** argv) {
    // 检查命令行参数
    if (argc != 2) {
        fmt::print(stderr, "使用方法: {} <图像路径>\n", argv[0]);
        fmt::print(stderr, "示例: {} ../img/test_image.jpg\n", argv[0]);
        return -1;
    }
    
    // 读取图像
    cv::Mat image = cv::imread(argv[1]);
    if (image.empty()) {
        fmt::print(stderr, "无法加载图像: {}\n", argv[1]);
        return -1;
    }
    
    fmt::print("成功加载图像: {}\n", argv[1]);
    
    // 缩放并居中图像
    cv::Mat result;
    ResizeParams params = resizeAndCenter(image, result);
    
    // 打印缩放参数
    printResizeParams(params);
    
    // 显示结果
    cv::imshow("原始图像", image);
    cv::imshow("缩放并居中后的图像", result);
    
    fmt::print("按任意键退出...\n");
    cv::waitKey(0);
    
    return 0;
}