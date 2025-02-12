#include "TrajectoryGenerator.hpp"
#include <cmath> // std::cos, std::sin
#include <chrono>


namespace TrajectoryGenerator {

    chai3d::cVector3d getCircularTrajectory(double radius,
        const chai3d::cVector3d& center,
        double angSpeed,
        double t)
    {
        // 由外部传入的角速度 (弧度/秒)
        double angle = angSpeed * t;

        // 根据圆周运动公式计算目标点坐标
        double x = center.x() + radius * std::cos(angle);
        double y = center.y() + radius * std::sin(angle);
        double z = center.z();  // 假设 z 坐标保持不变

        return chai3d::cVector3d(x, y, z);
    }

} // namespace TrajectoryGenerator
