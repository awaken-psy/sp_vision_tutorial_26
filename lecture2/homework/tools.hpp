#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <string>

// 图像缩放和居中参数结构体
struct ResizeParams {
    double scale;           // 缩放比例
    int offset_x;           // x方向偏移量
    int offset_y;           // y方向偏移量
    cv::Size original_size; // 原始图像尺寸
    cv::Size new_size;      // 缩放后图像尺寸
};

/**
 * @brief 将图像等比例缩放并居中放置在指定大小的画布上
 * 
 * @param input_image 输入图像
 * @param output_image 输出图像
 * @param canvas_size 画布大小 (默认640x640)
 * @return ResizeParams 包含缩放参数的结构体
 */
ResizeParams resizeAndCenter(const cv::Mat& input_image, cv::Mat& output_image, 
                            const cv::Size& canvas_size = cv::Size(640, 640));

/**
 * @brief 打印缩放参数
 * 
 * @param params 缩放参数结构体
 */
void printResizeParams(const ResizeParams& params);

#endif // TOOLS_HPP