#include "buff_solver.hpp"

namespace auto_buff
{

    std::vector<cv::Point3f> Buff_Solver::createObjectPoints()
    {
        std::vector<cv::Point3f> object_points;

        // 符的边缘点 - 确保顺序与检测到的关键点一致
        object_points.push_back(cv::Point3f(0, FAN_RADIUS, 0));  // 上方点
        object_points.push_back(cv::Point3f(FAN_RADIUS, 0, 0));  // 右方点
        object_points.push_back(cv::Point3f(0, -FAN_RADIUS, 0)); // 下方点
        object_points.push_back(cv::Point3f(-FAN_RADIUS, 0, 0)); // 左方点

        // 符中心 (原点)
        object_points.push_back(cv::Point3f(0, 0, 0));

        return object_points;
    }

    void Buff_Solver::solvePnP(const std::vector<cv::Point2f> &image_points, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, Solution &solution)
    {
        // 初始化结果为无效
        solution.valid = false;

        // 检查输入点数量是否足够（至少需要5个点用于PnP求解）
        if (image_points.size() < 5)
        {
            std::cout << "Not enough points for PnP: " << image_points.size() << std::endl;
            return;
        }

        // 创建对应的3D世界坐标系点
        std::vector<cv::Point3f> Objectpoints = createObjectPoints();

        // 确保2D图像点和3D世界点数量匹配，取较小值
        size_t num_points = std::min(image_points.size(), Objectpoints.size());
        std::vector<cv::Point2f> used_image_points(image_points.begin(), image_points.begin() + num_points);
        std::vector<cv::Point3f> used_Objectpoints(Objectpoints.begin(), Objectpoints.begin() + num_points);

        // 旋转向量和平移向量
        cv::Mat rvec, tvec;

        // 使用OpenCV的solvePnP函数求解相机姿态
        // 该函数通过2D-3D点对应关系计算相机相对于世界坐标系的位置和方向
        bool success = cv::solvePnP(used_Objectpoints, used_image_points, camera_matrix, dist_coeffs, rvec, tvec);

        if (!success)
        {
            std::cout << "PnP solve failed" << std::endl;
            return;
        }

        // 将旋转向量转换为旋转矩阵（3x3）
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        // 投影符中心和旋转中心到图像平面
        static float adjust_rmark_distance = R_MARK_DISTANCE; // 可调整的旋转中心距离
        std::vector<cv::Point3f> points3d;
        std::vector<cv::Point2f> points2d;
        points3d.push_back(cv::Point3f(0, 0, 0));                      // 符中心（世界坐标系原点）
        points3d.push_back(cv::Point3f(0, -adjust_rmark_distance, 0)); // 旋转中心（沿Y轴偏移）

        cv::projectPoints(points3d, rvec, tvec, camera_matrix, dist_coeffs, points2d);
        solution.fan_center = points2d[0];      // 获取符中心在图像上的投影坐标
        solution.rotation_center = points2d[1]; // 获取旋转中心在图像上的投影坐标

        // 新增：计算在相机坐标系下的三维坐标
        // 方法1：直接使用变换矩阵将世界坐标转换到相机坐标
        cv::Mat world_point_fan = (cv::Mat_<double>(3, 1) << 0, 0, 0);                           // 符中心世界坐标
        cv::Mat world_point_rotation = (cv::Mat_<double>(3, 1) << 0, -adjust_rmark_distance, 0); // 旋转中心世界坐标

        // 将世界坐标转换到相机坐标：P_camera = R * P_world + T
        cv::Mat camera_point_fan = rotation_matrix * world_point_fan + tvec;
        cv::Mat camera_point_rotation = rotation_matrix * world_point_rotation + tvec;

        // 存储三维坐标
        solution.fan_center3d = cv::Point3f(
            camera_point_fan.at<double>(0),
            camera_point_fan.at<double>(1),
            camera_point_fan.at<double>(2));

        solution.rotation_center3d = cv::Point3f(
            camera_point_rotation.at<double>(0),
            camera_point_rotation.at<double>(1),
            camera_point_rotation.at<double>(2));

        // 输出调试信息
        // std::cout << "PnP solve successful - Fan Center: " << solution.fan_center << ", Rotation Center: " << solution.rotation_center << std::endl;
        // std::cout << "tvec: [" << tvec.at<double>(0) << ", " << tvec.at<double>(1) << ", " << tvec.at<double>(2) << "]" << std::endl;
        // std::cout << "rvec: [" << rvec.at<double>(0) << ", " << rvec.at<double>(1) << ", " << rvec.at<double>(2) << "]" << std::endl;

        // 标记结果为有效
        solution.valid = true;

        // 调整距离
        static cv::Point2f last_pos = cv::Point2f(0.0f, 0.0f);
        cv::Point2f cur_pos = solution.rotation_center;

        static double last_dis = 10000000.0;
        double cur_dis = cv::norm(cur_pos - last_pos);

        static bool last_adjust = false; // false表示增加距离，true表示减小距离
        const double threshold = 1;      // 波动阈值
        const float step = 0.5f;         // 调整步长
        // static bool is_stable = false;

        // 稳定则不调整
        // if (is_stable)
        //     return;

        // 调整方向
        if (cur_dis >= last_dis)
            last_adjust = !last_adjust; // 反转操作方向

        // 调整步长
        if (!last_adjust)
            adjust_rmark_distance += step;
        else
            adjust_rmark_distance -= step;

        // 检查是否达到稳定状态
        // if (fabs(cur_dis - last_dis) < threshold)
        //     is_stable = true;

        // 更新上次的距离和位置
        last_dis = cur_dis;
        last_pos = cur_pos;
    }

} // namespace auto_buff