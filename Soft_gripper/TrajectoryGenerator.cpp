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
                double angleDeg = 360.0 / pointsEach * i;
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

} // namespace TrajectoryGenerator
