// 防止头文件重复包含
#ifndef TOOLS__LOGGER_HPP
#define TOOLS__LOGGER_HPP

#include <spdlog/spdlog.h>  // 高性能C++日志库

namespace tools
{
// 获取全局日志记录器实例
// 返回: spdlog日志记录器的共享指针，用于在整个项目中统一记录日志
std::shared_ptr<spdlog::logger> logger();

}  // namespace tools

#endif  // TOOLS__LOGGER_HPP