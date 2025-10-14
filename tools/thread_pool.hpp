// 防止头文件重复包含
#ifndef TOOLS__THREAD_POOL_HPP
#define TOOLS__THREAD_POOL_HPP

#include <condition_variable>  // 条件变量，用于线程同步
#include <functional>          // 函数对象支持
#include <mutex>               // 互斥锁
#include <queue>               // 队列容器
#include <thread>              // 线程支持
#include <vector>              // 向量容器

#include "tasks/auto_aim/yolo.hpp"  // YOLO检测器
#include "tools/logger.hpp"         // 日志工具

namespace tools
{
// 帧数据结构体：包含图像、姿态和检测结果
struct Frame
{
  int id;                                    // 帧ID（用于排序）
  cv::Mat img;                               // 图像数据
  std::chrono::steady_clock::time_point t;   // 时间戳
  Eigen::Quaterniond q;                      // 姿态四元数
  std::list<auto_aim::Armor> armors;         // 检测到的装甲板列表
};

// 创建多个YOLO11检测器的辅助函数
// config_path: 配置文件路径
// numebr: 创建的检测器数量
// debug: 是否开启调试模式
inline std::vector<auto_aim::YOLO> create_yolo11s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<auto_aim::YOLO> yolo11s;
  for (int i = 0; i < numebr; i++) {
    yolo11s.push_back(auto_aim::YOLO(config_path, debug));
  }
  return yolo11s;
}

// 创建多个YOLOv8检测器的辅助函数
// config_path: 配置文件路径
// numebr: 创建的检测器数量
// debug: 是否开启调试模式
inline std::vector<auto_aim::YOLO> create_yolov8s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<auto_aim::YOLO> yolov8s;
  for (int i = 0; i < numebr; i++) {
    yolov8s.push_back(auto_aim::YOLO(config_path, debug));
  }
  return yolov8s;
}

// 有序队列类：确保帧按ID顺序处理
class OrderedQueue
{
public:
  // 构造函数：初始化当前ID为1
  OrderedQueue() : current_id_(1) {}
  
  // 析构函数：清理队列和缓冲区
  ~OrderedQueue()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      main_queue_ = std::queue<tools::Frame>();
      buffer_.clear();
      current_id_ = 0;
    }
    tools::logger()->info("OrderedQueue destroyed, queue and buffer cleared.");
  }

  // 入队操作：按ID顺序管理帧
  void enqueue(const tools::Frame & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 检查ID是否小于当前期望ID
    if (item.id < current_id_) {
      tools::logger()->warn("small id");
      return;
    }

    // 如果ID正好是当前期望的ID，直接加入主队列
    if (item.id == current_id_) {
      main_queue_.push(item);
      current_id_++;

      // 检查缓冲区中是否有连续的后续帧
      auto it = buffer_.find(current_id_);
      while (it != buffer_.end()) {
        main_queue_.push(it->second);
        buffer_.erase(it);
        current_id_++;
        it = buffer_.find(current_id_);
      }

      // 通知等待的消费者
      if (main_queue_.size() >= 1) {
        cond_var_.notify_one();
      }
    } else {
      // ID大于当前期望ID，先放入缓冲区
      buffer_[item.id] = item;
    }
  }

  // 出队操作：阻塞直到有可用的帧
  tools::Frame dequeue()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待直到主队列不为空
    cond_var_.wait(lock, [this]() { return !main_queue_.empty(); });

    tools::Frame item = main_queue_.front();
    main_queue_.pop();
    return item;
  }

  // 尝试出队操作：非阻塞版本
  bool try_dequeue(tools::Frame & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (main_queue_.empty()) {
      return false;
    }
    item = main_queue_.front();
    main_queue_.pop();
    return true;
  }

  // 获取队列总大小（主队列+缓冲区）
  size_t get_size() { return main_queue_.size() + buffer_.size(); }

private:
  std::queue<tools::Frame> main_queue_;           // 主队列，按顺序存储帧
  std::unordered_map<int, tools::Frame> buffer_;  // 缓冲区，存储乱序到达的帧
  int current_id_;                                // 当前期望的帧ID
  std::mutex mutex_;                              // 互斥锁
  std::condition_variable cond_var_;              // 条件变量
};

// 线程池类：管理一组工作线程执行任务
class ThreadPool
{
public:
  // 构造函数：创建指定数量的工作线程
  ThreadPool(size_t num_threads) : stop(false)
  {
    for (size_t i = 0; i < num_threads; ++i) {
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(queue_mutex);
            // 等待直到有任务或线程池停止
            condition.wait(lock, [this] { return stop || !tasks.empty(); });
            if (stop && tasks.empty()) {
              return;
            }
            task = std::move(tasks.front());
            tasks.pop();
          }
          task();  // 执行任务
        }
      });
    }
  }

  // 析构函数：停止所有线程并清理资源
  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      stop = true;
      tasks = std::queue<std::function<void()>>();
    }
    condition.notify_all();
    for (std::thread & worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  // 添加任务到任务队列
  template <class F>
  void enqueue(F && f)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      if (stop) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }
      tasks.emplace(std::forward<F>(f));
    }
    condition.notify_one();  // 通知一个等待的线程
  }

private:
  std::vector<std::thread> workers;         // 工作线程集合
  std::queue<std::function<void()>> tasks;  // 任务队列
  std::mutex queue_mutex;                   // 任务队列互斥锁
  std::condition_variable condition;        // 条件变量，用于线程等待任务
  bool stop;                                // 线程池停止标志
};
}  // namespace tools

#endif  // TOOLS__THREAD_POOL_HPP