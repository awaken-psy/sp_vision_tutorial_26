#include <fmt/core.h>
#include <fmt/color.h>
#include <vector>
#include <string>

int main() {
    // 1. 基本格式化输出
    fmt::print("Hello, {}!\n", "fmt");
    
    // 2. 格式化数字
    int answer = 42;
    fmt::print("The answer is {}.\n", answer);
    
    // 3. 格式化浮点数
    double pi = 3.1415926535;
    fmt::print("Pi is approximately {:.2f}.\n", pi);
    
    // 4. 格式化多个值
    std::string name = "Alice";
    int age = 30;
    fmt::print("{} is {} years old.\n", name, age);
    
    // 5. 使用 format 函数返回字符串
    std::string message = fmt::format("{} + {} = {}", 2, 3, 2+3);
    fmt::print("{}\n", message);
    
    // 6. 带颜色的输出
    fmt::print(fg(fmt::color::red), "This is red text.\n");
    fmt::print(fg(fmt::color::green) | bg(fmt::color::blue), 
               "Green text on blue background.\n");
    
    // 7. 格式化容器（需要 C++17 或更高版本）
    std::vector<int> numbers = {1, 2, 3, 4, 5};
    fmt::print("Numbers: {}\n", fmt::join(numbers, ", "));
    
    // 8. 对齐文本
    fmt::print("{:<10} | {:^10} | {:>10}\n", "Left", "Center", "Right");
    fmt::print("{:<10} | {:^10} | {:>10}\n", 123, 456, 789);
    
    return 0;
}