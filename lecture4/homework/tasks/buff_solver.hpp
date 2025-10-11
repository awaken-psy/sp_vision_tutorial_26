#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <opencv2/opencv.hpp>

namespace auto_buff
{
    class Buff_Solver
    {
    public:
        // PnP解算结果
        struct Solution
        {
            cv::Point2f fan_center;      // 符中心位置（图像坐标）
            cv::Point2f rotation_center; // 旋转中心位置（图像坐标）
            bool valid = false;
        };
        
        void solvePnP(const std::vector<cv::Point2f>& image_points, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,Solution& solution);

        
    private:
        // 解算坐标
        std::vector<cv::Point3f> createObjectPoints(); // 创建符坐标系下的三维坐标

        // 能量机关尺寸
        static constexpr float FAN_RADIUS = 150.0f;      // 符半径
        static constexpr float R_MARK_DISTANCE = 700.0f; // 符中心到R标的距离
    };
} // namespace auto_buff
#endif // SOLVER_HPP