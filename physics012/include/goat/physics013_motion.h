#pragma once

#include <cstddef>
#include <vector>

namespace goat::physics013 {

struct MotionSample {
    double tSec = 0.0;
    double distance = 0.0;
};

struct RollingModel {
    double launchSpeed = 0.0;
    double deceleration = 0.0;
    double stopTime = 0.0;
    double stopDistance = 0.0;
    double rmse = 0.0;
    bool valid = false;
};

double speedAt(const RollingModel& model, double tSec);
double distanceAt(const RollingModel& model, double tSec);
RollingModel fitConstantDeceleration(const std::vector<MotionSample>& samples);

struct PowerPoint {
    double power = 0.0;      // normalized 0..1
    double launchSpeed = 0.0;
};

class PowerCurve {
public:
    explicit PowerCurve(std::vector<PowerPoint> points = {});
    bool valid() const;
    double launchSpeedFor(double normalizedPower) const;
    const std::vector<PowerPoint>& points() const { return points_; }

private:
    std::vector<PowerPoint> points_;
};

}  // namespace goat::physics013
