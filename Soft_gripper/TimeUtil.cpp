#include "TimeUtil.hpp"
#include <chrono>  

double getCurrentTime()
{
    // 1) 使用 steady_clock 避免系统时间调整的影响
    static const auto startTime = std::chrono::steady_clock::now();

    // 2) 获取当前时间点
    auto now = std::chrono::steady_clock::now();

    // 3) 计算时间差（秒）
    double seconds = std::chrono::duration<double>(now - startTime).count();
    
    return seconds;
}
