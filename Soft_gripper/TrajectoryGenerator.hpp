#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include "chai3d.h"
#include <vector>

namespace TrajectoryGenerator {

    // Generates a circular trajectory at time `t`:
    //   - radius (meters)
    //   - center (3D position)
    //   - angular speed (radians/sec)
    //   - time t (seconds)
    chai3d::cVector3d getCircularTrajectory(
        double radius,
        const chai3d::cVector3d& center,
        double angSpeed,
        double t
    );

    // Defines a circle with a given radius and center.
    struct CircleDefinition
    {
        double radius;
        chai3d::cVector3d center;
    };

    // Generates points on multiple circles:
    //   - `circles`   : list of circles (radius + center)
    //   - `pointsEach`: number of points per circle
    //
    // Returns a 2D container:
    //   - outer vector size = circles.size()
    //   - each inner vector size = pointsEach
    std::vector<std::vector<chai3d::cVector3d>>
        generateMultipleCirclesPoints(
            const std::vector<CircleDefinition>& circles,
            int pointsEach
        );

    // Smoothly transitions from `startPos` to `circlePos`
    // over the duration [0..duration], using linear interpolation (LERP).
    //   - currentTime is clamped to [0, duration].
    //   - returns the interpolated position.
    chai3d::cVector3d getTransitionPosition(
        const chai3d::cVector3d& startPos,
        const chai3d::cVector3d& circlePos,
        double currentTime,
        double duration
    );

} // namespace TrajectoryGenerator

#endif // TRAJECTORY_GENERATOR_HPP
