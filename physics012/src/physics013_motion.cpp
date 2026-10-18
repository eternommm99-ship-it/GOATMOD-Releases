#include "goat/physics013_motion.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace goat::physics013 {
namespace {

bool finiteNonNegative(double v) {
    return std::isfinite(v) && v >= 0.0;
}

double clamp01(double v) {
    return std::max(0.0, std::min(1.0, v));
}

}  // namespace

double speedAt(const RollingModel& model, double tSec) {
    if (!model.valid || !std::isfinite(tSec) || tSec <= 0.0) {
        return model.valid ? model.launchSpeed : 0.0;
    }
    return std::max(0.0, model.launchSpeed - model.deceleration * tSec);
}

double distanceAt(const RollingModel& model, double tSec) {
    if (!model.valid || !std::isfinite(tSec) || tSec <= 0.0) {
        return 0.0;
    }
    const double t = std::min(tSec, model.stopTime);
    return std::max(0.0, model.launchSpeed * t - 0.5 * model.deceleration * t * t);
}

RollingModel fitConstantDeceleration(const std::vector<MotionSample>& samples) {
    RollingModel out;
    if (samples.size() < 3) return out;

    // Fit d(t) = v0*t + b*t^2, with a = -2*b.
    double s11 = 0.0, s12 = 0.0, s22 = 0.0;
    double y1 = 0.0, y2 = 0.0;
    std::size_t n = 0;

    for (const auto& s : samples) {
        if (!std::isfinite(s.tSec) || !std::isfinite(s.distance) ||
            s.tSec <= 0.0 || s.distance < 0.0) {
            continue;
        }
        const double x1 = s.tSec;
        const double x2 = s.tSec * s.tSec;
        s11 += x1 * x1;
        s12 += x1 * x2;
        s22 += x2 * x2;
        y1 += x1 * s.distance;
        y2 += x2 * s.distance;
        ++n;
    }

    if (n < 3) return out;
    const double det = s11 * s22 - s12 * s12;
    if (!std::isfinite(det) || std::abs(det) < 1e-12) return out;

    const double v0 = (y1 * s22 - y2 * s12) / det;
    const double b  = (s11 * y2 - s12 * y1) / det;
    const double a = -2.0 * b;

    if (!finiteNonNegative(v0) || !std::isfinite(a) || a <= 0.0) return out;

    out.launchSpeed = v0;
    out.deceleration = a;
    out.stopTime = v0 / a;
    out.stopDistance = v0 * v0 / (2.0 * a);

    double sq = 0.0;
    std::size_t used = 0;
    for (const auto& s : samples) {
        if (!std::isfinite(s.tSec) || !std::isfinite(s.distance) ||
            s.tSec <= 0.0 || s.distance < 0.0) {
            continue;
        }
        const double t = std::min(s.tSec, out.stopTime);
        const double pred = std::max(0.0, v0 * t - 0.5 * a * t * t);
        const double e = pred - s.distance;
        sq += e * e;
        ++used;
    }
    out.rmse = used ? std::sqrt(sq / static_cast<double>(used)) : 0.0;
    out.valid = std::isfinite(out.stopTime) && std::isfinite(out.stopDistance) &&
                out.stopTime > 0.0 && out.stopDistance > 0.0;
    return out;
}

PowerCurve::PowerCurve(std::vector<PowerPoint> points) : points_(std::move(points)) {
    points_.erase(std::remove_if(points_.begin(), points_.end(), [](const PowerPoint& p) {
        return !std::isfinite(p.power) || !std::isfinite(p.launchSpeed) ||
               p.power < 0.0 || p.power > 1.0 || p.launchSpeed < 0.0;
    }), points_.end());

    std::sort(points_.begin(), points_.end(), [](const PowerPoint& a, const PowerPoint& b) {
        return a.power < b.power;
    });

    // Collapse duplicate power entries, keeping the largest speed to preserve monotonicity intent.
    std::vector<PowerPoint> compact;
    for (const auto& p : points_) {
        if (!compact.empty() && std::abs(compact.back().power - p.power) < 1e-9) {
            compact.back().launchSpeed = std::max(compact.back().launchSpeed, p.launchSpeed);
        } else {
            compact.push_back(p);
        }
    }
    points_ = std::move(compact);
}

bool PowerCurve::valid() const {
    if (points_.size() < 2) return false;
    for (std::size_t i = 1; i < points_.size(); ++i) {
        if (!(points_[i].power > points_[i - 1].power) ||
            points_[i].launchSpeed < points_[i - 1].launchSpeed) {
            return false;
        }
    }
    return true;
}

double PowerCurve::launchSpeedFor(double normalizedPower) const {
    if (!valid() || !std::isfinite(normalizedPower)) return 0.0;
    const double p = clamp01(normalizedPower);
    if (p <= points_.front().power) return points_.front().launchSpeed;
    if (p >= points_.back().power) return points_.back().launchSpeed;

    for (std::size_t i = 1; i < points_.size(); ++i) {
        const auto& a = points_[i - 1];
        const auto& b = points_[i];
        if (p <= b.power) {
            const double u = (p - a.power) / (b.power - a.power);
            return a.launchSpeed + u * (b.launchSpeed - a.launchSpeed);
        }
    }
    return points_.back().launchSpeed;
}

}  // namespace goat::physics013
