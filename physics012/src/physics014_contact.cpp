#include "goat/physics014_contact.h"

#include <algorithm>
#include <cmath>

namespace goat::physics014 {
namespace {

double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

Vec2 rotate(Vec2 v, double a) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

Vec2 tangent(Vec2 n) {
    return {-n.y, n.x};
}

}  // namespace

ContactResult solveEqualBallContact(const ContactInput& input, const ContactTuning& tuning) {
    ContactResult out;
    const Vec2 d = goat::physics012::normalized(input.incomingCueDir);
    const Vec2 n = goat::physics012::normalized(input.contactNormal);
    if (goat::physics012::norm2(d) <= goat::physics012::kEps ||
        goat::physics012::norm2(n) <= goat::physics012::kEps ||
        !std::isfinite(input.cueSpeed) || input.cueSpeed <= 0.0) {
        return out;
    }

    const double approach = goat::physics012::dot(d, n);
    if (!std::isfinite(approach) || approach <= 0.0) return out;

    const double e = clamp(tuning.restitution, 0.0, 1.0);
    const double side = clamp(input.sideSpin, -1.0, 1.0);
    const double top = clamp(input.topSpin, -1.0, 1.0);

    // Equal masses: ideal frictionless collision transfers the normal component
    // to the object ball; cue ball retains its tangential component.
    const Vec2 t = tangent(n);
    const double tangential = goat::physics012::dot(d, t);
    const double normalSpeed = input.cueSpeed * approach;
    const double tangentialSpeed = input.cueSpeed * tangential;

    const double cutFactor = std::sqrt(std::max(0.0, 1.0 - approach * approach));
    const double throwAngle = side * tuning.sideThrowRadians * cutFactor;
    out.objectDir = goat::physics012::normalized(rotate(n, throwAngle));
    out.objectSpeed = std::max(0.0, normalSpeed * e);

    Vec2 cueVelocity = t * tangentialSpeed + n * (normalSpeed * (1.0 - e));
    const double cueBaseSpeed = goat::physics012::norm(cueVelocity);
    if (cueBaseSpeed > goat::physics012::kEps) {
        out.cueDir = cueVelocity / cueBaseSpeed;
    }

    const double followMultiplier = std::max(0.0, 1.0 + top * tuning.followGain);
    out.cueSpeed = cueBaseSpeed * followMultiplier;
    out.appliedThrowRadians = throwAngle;
    out.valid = std::isfinite(out.objectSpeed) && std::isfinite(out.cueSpeed) &&
                std::isfinite(out.objectDir.x) && std::isfinite(out.objectDir.y) &&
                out.objectSpeed >= 0.0 && out.cueSpeed >= 0.0;
    return out;
}

}  // namespace goat::physics014
