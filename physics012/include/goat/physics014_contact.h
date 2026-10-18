#pragma once

#include "goat/physics012_geometry.h"

namespace goat::physics014 {

using goat::physics012::Vec2;

struct ContactTuning {
    double restitution = 1.0;          // 0..1, equal-ball normal transfer
    double sideThrowRadians = 0.0;     // calibrated later; 0 keeps ideal geometry
    double followGain = 0.0;           // top/back-spin cue-speed adjustment
};

struct ContactInput {
    Vec2 incomingCueDir{};             // cue travel direction
    Vec2 contactNormal{};              // cue-center -> object-center at impact
    double cueSpeed = 0.0;
    double sideSpin = 0.0;             // normalized -1..1
    double topSpin = 0.0;              // normalized -1..1
};

struct ContactResult {
    bool valid = false;
    Vec2 objectDir{};
    Vec2 cueDir{};
    double objectSpeed = 0.0;
    double cueSpeed = 0.0;
    double appliedThrowRadians = 0.0;
};

ContactResult solveEqualBallContact(const ContactInput& input, const ContactTuning& tuning = {});

}  // namespace goat::physics014
