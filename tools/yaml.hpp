// 防止头文件重复包含
#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

#include <yaml-cpp/yaml.h>  // YAML解析库

#include "tools/logger.hpp"  // 日志工具

namespace tools
{
// 加载YAML文件的辅助函数
// path: YAML文件路径
// 返回: 解析后的YAML节点
// 异常: 如果文件不存在或解析失败，记录错误并退出程序
inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::BadFile & e) {
    // 文件不存在或无法访问时的错误处理
    logger()->error("[YAML] Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException & e) {
    // YAML语法解析错误处理
    logger()->error("[YAML] Parser error: {}", e.what());
    exit(1);
  }
}

// 从YAML节点读取指定键值的模板函数
// T: 期望返回的数据类型
// yaml: YAML节点
// key: 要读取的键名
// 返回: 解析后的值
// 异常: 如果键不存在，记录错误并退出程序
template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  if (yaml[key]) return yaml[key].as<T>();
  // 键不存在时的错误处理
  logger()->error("[YAML] {} not found!", key);
  exit(1);
}

}  // namespace tools

#endif  // TOOLS__YAML_HPP