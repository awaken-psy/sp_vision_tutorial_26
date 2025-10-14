// 防止头文件重复包含
#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
// 退出检测器类：用于检测程序退出条件
// 通常用于在循环中检查是否应该退出程序
class Exiter
{
public:
  // 构造函数：初始化退出检测器
  Exiter();

  // 检查是否应该退出程序
  // 返回: 如果需要退出返回true，否则返回false
  bool exit() const;
};

}  // namespace tools

#endif  // TOOLS__EXITER_HPP