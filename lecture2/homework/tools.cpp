#include "tools.hpp"
#include <fmt/core.h>
#include <opencv2/opencv.hpp>

ResizeParams resizeAndCenter(const cv::Mat& input_image, cv::Mat& output_image, 
                            const cv::Size& canvas_size) {
    ResizeParams params;
    params.original_size = input_image.size();
    
    // 计算缩放比例
    double scale_x = static_cast<double>(canvas_size.width) / input_image.cols;
    double scale_y = static_cast<double>(canvas_size.height) / input_image.rows;
    params.scale = std::min(scale_x, scale_y); // 取较小比例以确保图像完全放入画布
    
    // 计算缩放后的图像尺寸
    int new_width = static_cast<int>(input_image.cols * params.scale);
    int new_height = static_cast<int>(input_image.rows * params.scale);
    params.new_size = cv::Size(new_width, new_height);
    
    // 计算偏移量以使图像居中
    params.offset_x = (canvas_size.width - new_width) / 2;
    params.offset_y = (canvas_size.height - new_height) / 2;
    
    // 缩放图像
    cv::Mat resized_image;
    cv::resize(input_image, resized_image, params.new_size, 0, 0, cv::INTER_LINEAR);
    
    // 创建黑色画布
    output_image = cv::Mat::zeros(canvas_size, input_image.type());
    
    // 将缩放后的图像复制到画布中央
    cv::Mat roi(output_image, 
               cv::Rect(params.offset_x, params.offset_y, new_width, new_height));
    resized_image.copyTo(roi);
    
    return params;
}

void printResizeParams(const ResizeParams& params) {
    fmt::print("=== 图像缩放参数 ===\n");
    fmt::print("原始尺寸: {}x{}\n", params.original_size.width, params.original_size.height);
    fmt::print("缩放比例: {:.4f}\n", params.scale);
    fmt::print("缩放后尺寸: {}x{}\n", params.new_size.width, params.new_size.height);
    fmt::print("X方向偏移: {} 像素\n", params.offset_x);
    fmt::print("Y方向偏移: {} 像素\n", params.offset_y);
    fmt::print("画布尺寸: 640x640\n");
}