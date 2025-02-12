#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include "chai3d.h"

namespace TrajectoryGenerator {

    // 生成圆形轨迹，参数：
    // - radius:   圆形轨迹半径
    // - center:   圆心坐标
    // - angSpeed: 角速度 (单位: 弧度/秒)
    // - t:        时间 (秒)
    chai3d::cVector3d getCircularTrajectory(double radius,
        const chai3d::cVector3d& center,
        double angSpeed,
        double t);
}

#endif // TRAJECTORY_GENERATOR_HPP
