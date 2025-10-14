// 防止头文件重复包含
#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>  // OpenCV计算机视觉库
#include <string>
#include <vector>

namespace tools
{
// 在图像上绘制单个点（圆形标记）
// img: 目标图像
// point: 要绘制的点坐标
// color: 点的颜色（BGR格式，默认红色）
// radius: 点的半径（默认3像素）
void draw_point(
  cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3);

// 在图像上绘制多个点（整数坐标版本）
// img: 目标图像
// points: 要绘制的点坐标向量
// color: 点的颜色（BGR格式，默认红色）
// thickness: 点的线宽（默认2像素）
void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

// 在图像上绘制多个点（浮点坐标版本）
// img: 目标图像
// points: 要绘制的点坐标向量（浮点数）
// color: 点的颜色（BGR格式，默认红色）
// thickness: 点的线宽（默认2像素）
void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

// 在图像上绘制文本
// img: 目标图像
// text: 要绘制的文本内容
// point: 文本起始位置（左下角坐标）
// color: 文本颜色（BGR格式，默认黄色）
// font_scale: 字体大小缩放因子（默认1.0）
// thickness: 文本线宽（默认2像素）
void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point,
  const cv::Scalar & color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2);

}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP