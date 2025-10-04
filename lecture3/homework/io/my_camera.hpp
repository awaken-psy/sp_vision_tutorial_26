#ifndef MY_CAMERA_HPP
#define MY_CAMERA_HPP

// 使用正确的包含方式
#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"

class myCamera
{
public:
    // 构造函数
    myCamera();
    
    // 析构函数
    ~myCamera();
    
    // 读取一帧图像
    bool read(cv::Mat& frame);

private:
    void* handle_;                    // 设备句柄
    MV_CC_DEVICE_INFO_LIST device_list_; // 设备列表
    bool is_initialized_;             // 初始化标志
    
    // 图像转换函数
    cv::Mat transfer_(MV_FRAME_OUT& raw);
};

#endif // MY_CAMERA_HPP