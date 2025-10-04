#include "my_camera.hpp"
#include <iostream>
#include <unordered_map>

myCamera::myCamera() : handle_(nullptr), is_initialized_(false)
{
    // 枚举设备
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);
    if (ret != MV_OK) {
        std::cout << "Enum devices failed! ret = " << ret << std::endl;
        return;
    }
    
    if (device_list_.nDeviceNum == 0) {
        std::cout << "No camera found!" << std::endl;
        return;
    }
    
    // 创建设备句柄
    ret = MV_CC_CreateHandle(&handle_, device_list_.pDeviceInfo[0]);
    if (ret != MV_OK) {
        std::cout << "Create handle failed! ret = " << ret << std::endl;
        return;
    }
    
    // 打开设备
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        std::cout << "Open device failed! ret = " << std::hex << ret << std::endl;
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return;
    }
    
    // 设置相机参数（与example.cpp保持一致）
    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", 10000);
    MV_CC_SetFloatValue(handle_, "Gain", 20);
    MV_CC_SetFrameRate(handle_, 60);
    
    // 开始采集图像
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        std::cout << "Start grabbing failed! ret = " << std::hex << ret << std::endl;
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return;
    }
    
    is_initialized_ = true;
    std::cout << "Camera initialized successfully!" << std::endl;
}

myCamera::~myCamera()
{
    if (handle_ != nullptr) {
        if (is_initialized_) {
            // 停止采集
            MV_CC_StopGrabbing(handle_);
        }
        
        // 关闭设备
        MV_CC_CloseDevice(handle_);
        
        // 销毁句柄
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
    
    std::cout << "Camera released!" << std::endl;
}

cv::Mat myCamera::transfer_(MV_FRAME_OUT& raw)
{
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight;

    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
      {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
      {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
      {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
      {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
    cv::cvtColor(img, img, type_map.at(pixel_type));
    
    return img;
}

bool myCamera::read(cv::Mat& frame)
{
    if (!is_initialized_ || handle_ == nullptr) {
        std::cout << "Camera not initialized!" << std::endl;
        return false;
    }
    
    MV_FRAME_OUT raw;
    unsigned int nMsec = 100;

    int ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
    if (ret != MV_OK) {
        std::cout << "Get image buffer failed! ret = " << std::hex << ret << std::endl;
        return false;
    }

    // 使用transfer_函数转换图像
    frame = transfer_(raw);

    ret = MV_CC_FreeImageBuffer(handle_, &raw);
    if (ret != MV_OK) {
        std::cout << "Free image buffer failed! ret = " << std::hex << ret << std::endl;
        return false;
    }
    
    return true;
}