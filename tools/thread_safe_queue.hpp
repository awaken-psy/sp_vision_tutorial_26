// 防止头文件重复包含
#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>  // 条件变量，用于线程间同步
#include <functional>          // 函数对象支持
#include <iostream>
#include <mutex>               // 互斥锁
#include <queue>               // 标准队列容器

namespace tools
{
// 线程安全队列模板类
// T: 队列元素类型
// PopWhenFull: 队列满时的处理策略，true表示弹出最旧元素，false表示拒绝新元素
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  // 构造函数：设置队列最大容量和队列满时的处理函数
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }

  // 向队列推送元素（线程安全）
  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // 检查队列是否已满
    if (queue_.size() >= max_size_) {
      if (PopWhenFull) {
        // 队列满时弹出最旧元素（先进先出）
        queue_.pop();
      } else {
        // 队列满时调用处理函数并返回，不添加新元素
        full_handler_();
        return;
      }
    }

    // 添加新元素并通知等待的消费者线程
    queue_.push(value);
    not_empty_condition_.notify_all();
  }

  // 从队列弹出元素（引用参数版本，线程安全）
  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待直到队列不为空
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    // 安全检查：确保队列不为空
    if (queue_.empty()) {
      std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
      return;
    }

    // 获取并移除队列前端元素
    value = queue_.front();
    queue_.pop();
  }

  // 从队列弹出元素（返回值版本，线程安全）
  T pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待直到队列不为空
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    // 使用移动语义获取并移除队列前端元素
    T value = std::move(queue_.front());
    queue_.pop();
    return std::move(value);
  }

  // 获取队列前端元素（不移除，线程安全）
  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待直到队列不为空
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    return queue_.front();
  }

  // 获取队列后端元素（不移除，线程安全）
  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // 安全检查：确保队列不为空
    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  // 检查队列是否为空（线程安全）
  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  // 清空队列（线程安全）
  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
    // 通知所有等待队列不为空的线程
    not_empty_condition_.notify_all();
  }

private:
  std::queue<T> queue_;                    // 底层队列容器
  size_t max_size_;                        // 队列最大容量
  mutable std::mutex mutex_;               // 互斥锁，保证线程安全
  std::condition_variable not_empty_condition_;  // 条件变量，用于等待队列非空
  std::function<void(void)> full_handler_; // 队列满时的处理函数
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP