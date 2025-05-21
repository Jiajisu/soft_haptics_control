#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include "chai3d.h"
#include <vector>

namespace TrajectoryGenerator {

    // 生成一个圆形轨迹
    //   - radius: 圆的半径
    //   - center: 圆心（3D坐标）
    //   - angSpeed: 角速度（弧度/秒）
    //   - t: 时间（秒）
    chai3d::cVector3d getCircularTrajectory(
        double radius,
        const chai3d::cVector3d& center,
        double angSpeed,
        double t
    );

    // 圆形定义
    struct CircleDefinition
    {
        double radius;
        chai3d::cVector3d center;
    };

    // 生成多个圆形上的离散点：
    //   - circles: 多个圆形 (半径 + 圆心)
    //   - pointsEach: 每个圆上要生成的点数量
    //
    // 返回值为一个二维容器：
    //   - 外层 vector 大小 = circles.size()
    //   - 每个内层 vector 大小 = pointsEach
    std::vector<std::vector<chai3d::cVector3d>>
        generateMultipleCirclesPoints(
            const std::vector<CircleDefinition>& circles,
            int pointsEach
        );

    // 从 startPos 平滑过渡到 circlePos，
    // 在时长 [0..duration] 内使用线性插值(LERP)。
    //   - currentTime 会被限制在 [0, duration] 之间
    //   - 返回该时刻插值后的位置
    chai3d::cVector3d getTransitionPosition(
        const chai3d::cVector3d& startPos,
        const chai3d::cVector3d& circlePos,
        double currentTime,
        double duration
    );

    // 生成从 startPos 到 endPos 的直线轨迹
    // 在时长 [0..duration] 内使用线性插值(LERP)。
    //   - currentTime 会被限制在 [0, duration] 之间
    //   - 返回该时刻插值后的位置
    chai3d::cVector3d getStraightLineTrajectory(
        const chai3d::cVector3d& startPos,
        const chai3d::cVector3d& endPos,
        double currentTime,
        double duration
    );

    chai3d::cVector3d getSineWaveTrajectory(
        const chai3d::cVector3d& center,        // 基准点
        const chai3d::cVector3d& amplitude,     // 各轴峰值 (m, mm 任意单位)
        double freq,                            // 频率  Hz
        double t,                               // 当前时间  s
        int numPeriods = 11                     // 运行周期数，默认 11
    );

} // namespace TrajectoryGenerator

#endif // TRAJECTORY_GENERATOR_HPP
