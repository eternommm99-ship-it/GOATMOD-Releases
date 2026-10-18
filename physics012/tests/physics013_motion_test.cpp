#include "goat/physics013_motion.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace goat::physics013;

namespace {

bool near(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

void testSyntheticFit() {
    constexpr double v0 = 120.0;
    constexpr double a = 30.0;
    std::vector<MotionSample> samples;
    for (int i = 1; i <= 8; ++i) {
        const double t = 0.25 * i;
        samples.push_back({t, v0 * t - 0.5 * a * t * t});
    }
    const auto fit = fitConstantDeceleration(samples);
    assert(fit.valid);
    assert(near(fit.launchSpeed, v0, 1e-8));
    assert(near(fit.deceleration, a, 1e-8));
    assert(near(fit.stopTime, 4.0, 1e-8));
    assert(near(fit.stopDistance, 240.0, 1e-8));
    assert(fit.rmse < 1e-8);
}

void testStopClamp() {
    RollingModel m;
    m.launchSpeed = 100.0;
    m.deceleration = 20.0;
    m.stopTime = 5.0;
    m.stopDistance = 250.0;
    m.valid = true;
    assert(near(speedAt(m, 2.0), 60.0));
    assert(near(speedAt(m, 10.0), 0.0));
    assert(near(distanceAt(m, 5.0), 250.0));
    assert(near(distanceAt(m, 10.0), 250.0));
}

void testRejectInvalidFit() {
    assert(!fitConstantDeceleration({{0.1, 1.0}, {0.2, 2.0}}).valid);
    assert(!fitConstantDeceleration({{0.1, 1.0}, {0.2, 2.0}, {0.3, 3.0}}).valid);
}

void testPowerInterpolation() {
    PowerCurve curve({{0.0, 0.0}, {0.5, 100.0}, {1.0, 220.0}});
    assert(curve.valid());
    assert(near(curve.launchSpeedFor(0.25), 50.0));
    assert(near(curve.launchSpeedFor(0.75), 160.0));
    assert(near(curve.launchSpeedFor(-1.0), 0.0));
    assert(near(curve.launchSpeedFor(2.0), 220.0));
}

void testPowerRejectsNonMonotonic() {
    PowerCurve curve({{0.0, 0.0}, {0.5, 120.0}, {1.0, 100.0}});
    assert(!curve.valid());
    assert(near(curve.launchSpeedFor(0.5), 0.0));
}

}  // namespace

int main() {
    testSyntheticFit();
    testStopClamp();
    testRejectInvalidFit();
    testPowerInterpolation();
    testPowerRejectsNonMonotonic();
    std::cout << "GOATMOD_PHYSICS_013_MOTION_TEST=PASS\n";
    return 0;
}
