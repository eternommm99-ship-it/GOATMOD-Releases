#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace goat::physics012 {

constexpr double kEps = 1e-9;

struct Vec2 {
    double x{};
    double y{};
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline Vec2 operator/(Vec2 a, double s) { return {a.x / s, a.y / s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double norm2(Vec2 a) { return dot(a, a); }
inline double norm(Vec2 a) { return std::sqrt(norm2(a)); }
inline Vec2 normalized(Vec2 a) {
    const double n = norm(a);
    return n > kEps ? a / n : Vec2{};
}

struct Rect {
    double left{};
    double top{};
    double right{};
    double bottom{};
};

enum class Rail { Left, Right, Top, Bottom };

enum class PocketKind { Corner, Side };

struct Pocket {
    Vec2 center{};
    PocketKind kind{PocketKind::Corner};
    double mouthHalfWidth{};
};

struct Ball {
    int id{};
    Vec2 center{};
    double radius{};
};

struct CushionHit {
    Rail rail{};
    Vec2 ballCenterAtImpact{};
    Vec2 cushionContactPoint{};
    Vec2 normal{};
    double distance{};
};

struct Segment {
    Vec2 a{};
    Vec2 b{};
};

struct BankRoute {
    bool valid{false};
    Rail rail{Rail::Left};
    Vec2 bankCenter{};
    Vec2 bankContact{};
    Vec2 outgoingDir{};
};

// Playable center rectangle: physical cloth/cushion bounds inset by ball radius.
Rect insetForBallCenter(Rect physicalBounds, double ballRadius);

// Ideal specular reflection. d and n need not be normalized.
Vec2 reflect(Vec2 d, Vec2 n);

// First impact against playable center bounds. Returns nullopt for zero direction.
std::optional<CushionHit> firstCushionHit(
    Vec2 origin,
    Vec2 direction,
    Rect physicalBounds,
    double ballRadius);

// True when impact falls inside a detected pocket mouth on the corresponding rail.
bool impactInsidePocketMouth(
    const CushionHit& hit,
    const std::array<Pocket, 6>& pockets);

// Distance from p to finite segment ab.
double pointSegmentDistance(Vec2 p, Vec2 a, Vec2 b);

// Obstacle corridor check. clearanceRadius should normally be movingRadius + obstacleRadius.
bool segmentBlockedByBalls(
    Segment path,
    const std::vector<Ball>& obstacles,
    double movingRadius,
    int ignoreIdA = -1,
    int ignoreIdB = -1,
    double extraMargin = 0.0);

// Geometric pocket corridor only; this does not claim guaranteed potting physics.
bool segmentReachesPocketMouth(
    Segment path,
    const Pocket& pocket,
    Rail approachRail,
    double tolerance = 1.0);

// One-bank route using mirror geometry. Rejects nonexistent rail inside pocket mouths.
BankRoute computeOneBankRoute(
    Vec2 start,
    Vec2 destination,
    Rail rail,
    Rect physicalBounds,
    double ballRadius,
    const std::array<Pocket, 6>& pockets);

}  // namespace goat::physics012
