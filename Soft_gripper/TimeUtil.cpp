#include "TimeUtil.hpp"
#include <chrono>  // std::chrono

double getCurrentTime()
{
    // 用 static 变量保存程序启动时的时间点，这样后续每次调用
    // 都会以同一基准（startTime）计算相对时间
    static const auto startTime = std::chrono::system_clock::now();

    auto now = std::chrono::system_clock::now();
    // 计算从 startTime 到现在的时间间隔
    auto duration = now - startTime;
    // 将间隔转换为以微秒为单位的 double 数值，再除以 1e6 得到秒数
    double seconds = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(duration).count() / 1e6;
    return seconds;
}
