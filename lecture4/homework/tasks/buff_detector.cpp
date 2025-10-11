#include "buff_detector.hpp"


namespace auto_buff
{
Buff_Detector::Buff_Detector() : MODE_() {}


/**
 * @brief 检测图像中的能量机关扇叶
 * 
 * 使用YOLO模型检测图像中的扇叶目标，并将检测结果转换为FanBlade结构
 * 
 * @param bgr_img 输入图像（BGR格式）
 * @return std::vector<FanBlade> 检测到的扇叶目标列表
 */
std::vector<FanBlade> Buff_Detector::detect(cv::Mat & bgr_img)
{
    // 1. 使用YOLO模型获取图像中的候选检测框
    // YOLO11_BUFF::Object 包含目标框、关键点等信息
    std::vector<YOLO11_BUFF::Object> results = MODE_.get_onecandidatebox(bgr_img);
    
    // 2. 检查是否有检测结果
    if (results.empty()) {
        // 如果没有检测到任何目标，返回空向量
        return std::vector<FanBlade>();
    }
    
    // 3. 创建扇叶容器
    std::vector<FanBlade> fanblades;
    
    // 4. 获取第一个检测结果（通常假设图像中只有一个主要目标）
    auto result = results[0];
    
    // 5. 将YOLO检测结果转换为FanBlade结构
    // 参数说明：
    // - result.kpt: YOLO检测到的关键点坐标数组
    // - result.kpt[4]: 第5个关键点作为扇叶中心点（索引从0开始）
    // - _light: 扇叶类型，这里固定为亮扇叶类型
    fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));
    
    // 6. 返回检测到的扇叶列表
    return fanblades;
}
}  // namespace auto_buff