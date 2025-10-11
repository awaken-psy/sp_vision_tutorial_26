#ifndef BUFF__TYPE_HPP
#define BUFF__TYPE_HPP

#include <algorithm>
#include <deque>
#include <eigen3/Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>
namespace auto_buff
{
// 定义常量：表示一个很大的数值，用于初始化或比较
const int INF = 1000000;

// 能量机关类型枚举
enum PowerRune_type { 
    SMALL,  // 小能量机关
    BIG     // 大能量机关
};

// 扇叶类型枚举
enum FanBlade_type { 
    _target,   // 目标扇叶（需要击打的扇叶）
    _unlight,  // 未点亮的扇叶
    _light     // 已点亮的扇叶
};

// 跟踪状态枚举
enum Track_status { 
    TRACK,     // 正在跟踪
    TEM_LOSE,  // 暂时丢失目标
    LOSE       // 完全丢失目标
};

/**
 * @brief 扇叶类，表示能量机关中的一个扇叶目标
 * 
 * 该类封装了扇叶的几何属性、位置信息和状态类型，
 * 用于能量机关的检测和跟踪
 */
class FanBlade
{
public:
    // 几何和位置属性
    cv::Point2f center;               // 扇叶中心点坐标
    std::vector<cv::Point2f> points;  // 扇叶关键点坐标集合（通常为角点）
    double angle, width, height;      // 扇叶的角度、宽度和高度
    
    // 状态属性
    FanBlade_type type;               // 扇叶类型（目标/未点亮/已点亮）

    /**
     * @brief 默认构造函数
     * 
     * 使用默认值初始化所有成员变量
     */
    explicit FanBlade() = default;

    /**
     * @brief 通过关键点和中心点构造扇叶
     * 
     * @param kpt 关键点坐标向量，表示扇叶的轮廓或特征点
     * @param keypoints_center 扇叶的中心点坐标
     * @param t 扇叶类型
     */
    explicit FanBlade(
        const std::vector<cv::Point2f> & kpt, 
        cv::Point2f keypoints_center, 
        FanBlade_type t);

    /**
     * @brief 通过类型构造扇叶
     * 
     * 仅初始化扇叶类型，其他属性使用默认值
     * 
     * @param t 扇叶类型
     */
    explicit FanBlade(FanBlade_type t);
};
}
#endif  // BUFF_TYPE_HPP
