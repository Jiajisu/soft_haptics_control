#include "TrajectoryGenerator.hpp"
#include <cmath>    // for std::cos, std::sin, M_PI
#include <chrono>

namespace TrajectoryGenerator {

    chai3d::cVector3d getCircularTrajectory(
        double radius,
        const chai3d::cVector3d& center,
        double angSpeed,
        double t
    )
    {
        double angle = angSpeed * t;
        double x = center.x() + radius * std::cos(angle);
        double y = center.y() + radius * std::sin(angle);
        double z = center.z(); // keep Z unchanged
        return chai3d::cVector3d(x, y, z);
    }

    std::vector<std::vector<chai3d::cVector3d>>
        generateMultipleCirclesPoints(
            const std::vector<CircleDefinition>& circles,
            int pointsEach
        )
    {   
        constexpr double kStartDegOffset =  -30;   // 统一起始角偏移 15°

        std::vector<std::vector<chai3d::cVector3d>> result;
        result.resize(circles.size());

        for (size_t c = 0; c < circles.size(); ++c)
        {
            double radius = circles[c].radius;
            chai3d::cVector3d center = circles[c].center;
            result[c].resize(pointsEach);

            for (int i = 0; i < pointsEach; ++i)
            {
                // Similar to MATLAB: angle = 2pi*(i / pointsEach)
                double angleDeg = kStartDegOffset + 360.0 / pointsEach * i;
                double angleRad = angleDeg * M_PI / 180.0;

                double xPos = center.x() + radius * std::cos(angleRad);
                double yPos = center.y() + radius * std::sin(angleRad);
                double zPos = center.z();

                result[c][i] = chai3d::cVector3d(xPos, yPos, zPos);
            }
        }
        return result;
    }

    chai3d::cVector3d getTransitionPosition(
        const chai3d::cVector3d& startPos,
        const chai3d::cVector3d& circlePos,
        double currentTime,
        double duration
    )
    {
        double alpha = 0.0;
        if (duration > 1e-9)
        {
            alpha = currentTime / duration;
        }
        if (alpha < 0.0) alpha = 0.0;
        if (alpha > 1.0) alpha = 1.0;

        return (1.0 - alpha) * startPos + alpha * circlePos;
    }

    chai3d::cVector3d getStraightLineTrajectory(
        const chai3d::cVector3d& startPos,
        const chai3d::cVector3d& endPos,
        double currentTime,
        double duration
    )
    {
        // 计算插值系数 alpha，范围在 [0, 1]
        double alpha = 0.0;
        if (duration > 1e-9)
        {
            alpha = currentTime / duration;
        }
        if (alpha < 0.0) alpha = 0.0;
        if (alpha > 1.0) alpha = 1.0;

        // 利用线性插值进行位置计算
        return (1.0 - alpha) * startPos + alpha * endPos;
    }

    chai3d::cVector3d getSineWaveTrajectory(
        const chai3d::cVector3d& center,
        const chai3d::cVector3d& amplitude,
        double freq,
        double t,
        int numPeriods)
    {
        if (freq <= 1e-9) {          // 频率异常保护
            return center;
        }

        // 总时长 = 周期 × 数量
        const double totalDuration = static_cast<double>(numPeriods) / freq;

        // 超出时长 —— 直接停在中心点
        if (t >= totalDuration) {
            return center;
        }

        const double angle = 2.0 * M_PI * freq * t;   // ωt  (rad)

        const double x = center.x() + amplitude.x() * std::sin(angle);
        const double y = center.y() + amplitude.y() * std::sin(angle);
        const double z = center.z() + amplitude.z() * std::sin(angle);

        return chai3d::cVector3d(x, y, z);
    }

} // namespace TrajectoryGenerator
