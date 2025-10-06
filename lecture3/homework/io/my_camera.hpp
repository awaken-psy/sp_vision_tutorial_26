#ifndef MY_CAMERA_HPP
#define MY_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"

class myCamera {
public:
    // 构造函数
    myCamera();
    
    // 析构函数
    ~myCamera();
    
    // 读取一帧图像
    cv::Mat read();

private:
    // 私有成员变量以_结尾
    void* handle_;
    bool is_initialized_;
    bool is_grabbing_;
    
    // 图像转换函数
    cv::Mat transfer_(MV_FRAME_OUT& raw);
};

#endif // MY_CAMERA_HPP