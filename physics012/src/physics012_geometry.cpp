#include "goat/physics012_geometry.h"

#include <algorithm>
#include <limits>

namespace goat::physics012 {
namespace {
Vec2 railNormal(Rail rail) {
    switch (rail) {
        case Rail::Left: return {1.0, 0.0};
        case Rail::Right: return {-1.0, 0.0};
        case Rail::Top: return {0.0, 1.0};
        case Rail::Bottom: return {0.0, -1.0};
    }
    return {};
}

double railPlane(const Rect& r, Rail rail) {
    switch (rail) {
        case Rail::Left: return r.left;
        case Rail::Right: return r.right;
        case Rail::Top: return r.top;
        case Rail::Bottom: return r.bottom;
    }
    return 0.0;
}

bool centerOnRailSpan(Vec2 p, const Rect& r, Rail rail) {
    switch (rail) {
        case Rail::Left:
        case Rail::Right:
            return p.y >= r.top - kEps && p.y <= r.bottom + kEps;
        case Rail::Top:
        case Rail::Bottom:
            return p.x >= r.left - kEps && p.x <= r.right + kEps;
    }
    return false;
}

Vec2 mirrorAcrossRail(Vec2 p, Rail rail, double plane) {
    switch (rail) {
        case Rail::Left:
        case Rail::Right: p.x = 2.0 * plane - p.x; break;
        case Rail::Top:
        case Rail::Bottom: p.y = 2.0 * plane - p.y; break;
    }
    return p;
}
}

Rect insetForBallCenter(Rect b, double r) {
    return {b.left + r, b.top + r, b.right - r, b.bottom - r};
}

Vec2 reflect(Vec2 d, Vec2 n) {
    const Vec2 dn = normalized(d);
    const Vec2 nn = normalized(n);
    if (norm2(dn) <= kEps || norm2(nn) <= kEps) return {};
    return normalized(dn - nn * (2.0 * dot(dn, nn)));
}

std::optional<CushionHit> firstCushionHit(Vec2 origin, Vec2 direction, Rect physicalBounds, double ballRadius) {
    const Vec2 d = normalized(direction);
    if (norm2(d) <= kEps) return std::nullopt;
    const Rect r = insetForBallCenter(physicalBounds, ballRadius);
    if (r.left > r.right || r.top > r.bottom) return std::nullopt;

    struct Candidate { Rail rail; double t; };
    std::array<Candidate,4> c{{
        {Rail::Left, d.x < -kEps ? (r.left-origin.x)/d.x : std::numeric_limits<double>::infinity()},
        {Rail::Right,d.x >  kEps ? (r.right-origin.x)/d.x : std::numeric_limits<double>::infinity()},
        {Rail::Top, d.y < -kEps ? (r.top-origin.y)/d.y : std::numeric_limits<double>::infinity()},
        {Rail::Bottom,d.y > kEps ? (r.bottom-origin.y)/d.y : std::numeric_limits<double>::infinity()}
    }};

    double bestT = std::numeric_limits<double>::infinity();
    Rail bestRail = Rail::Left;
    Vec2 bestCenter{};
    for (const auto& x: c) {
        if (!(x.t > kEps) || !std::isfinite(x.t)) continue;
        Vec2 p = origin + d*x.t;
        if (!centerOnRailSpan(p,r,x.rail)) continue;
        if (x.t < bestT) { bestT=x.t; bestRail=x.rail; bestCenter=p; }
    }
    if (!std::isfinite(bestT)) return std::nullopt;
    const Vec2 n = railNormal(bestRail);
    return CushionHit{bestRail,bestCenter,bestCenter-n*ballRadius,n,bestT};
}

bool impactInsidePocketMouth(const CushionHit& hit, const std::array<Pocket,6>& pockets) {
    for (const auto& p: pockets) {
        if (!(p.mouthHalfWidth > 0.0)) continue;
        const Vec2 d = hit.cushionContactPoint - p.center;
        double tangent=0.0, normal=0.0;
        switch (hit.rail) {
            case Rail::Left:
            case Rail::Right: tangent=std::abs(d.y); normal=std::abs(d.x); break;
            case Rail::Top:
            case Rail::Bottom: tangent=std::abs(d.x); normal=std::abs(d.y); break;
        }
        if (tangent <= p.mouthHalfWidth + kEps && normal <= p.mouthHalfWidth*1.5 + kEps) return true;
    }
    return false;
}

double pointSegmentDistance(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab=b-a;
    const double den=norm2(ab);
    if (den <= kEps) return norm(p-a);
    const double t=std::clamp(dot(p-a,ab)/den,0.0,1.0);
    return norm(p-(a+ab*t));
}

bool segmentBlockedByBalls(Segment path, const std::vector<Ball>& obstacles, double movingRadius, int ignoreIdA, int ignoreIdB, double extraMargin) {
    if (norm2(path.b-path.a) <= kEps) return false;
    for (const auto& b: obstacles) {
        if (b.id==ignoreIdA || b.id==ignoreIdB) continue;
        const double clearance=std::max(0.0,movingRadius+b.radius+extraMargin);
        if (pointSegmentDistance(b.center,path.a,path.b) < clearance-kEps) return true;
    }
    return false;
}

bool segmentReachesPocketMouth(Segment path, const Pocket& pocket, Rail approachRail, double tolerance) {
    const double half=std::max(0.0,pocket.mouthHalfWidth+tolerance);
    const Vec2 d=path.b-pocket.center;
    switch (approachRail) {
        case Rail::Left:
        case Rail::Right: return std::abs(d.y) <= half;
        case Rail::Top:
        case Rail::Bottom: return std::abs(d.x) <= half;
    }
    return false;
}

BankRoute computeOneBankRoute(Vec2 start, Vec2 destination, Rail rail, Rect physicalBounds, double ballRadius, const std::array<Pocket,6>& pockets) {
    const Rect r=insetForBallCenter(physicalBounds,ballRadius);
    const double plane=railPlane(r,rail);
    const Vec2 mirrored=mirrorAcrossRail(destination,rail,plane);
    const Vec2 delta=mirrored-start;
    double t=-1.0;
    switch (rail) {
        case Rail::Left:
        case Rail::Right:
            if (std::abs(delta.x)<=kEps) return {};
            t=(plane-start.x)/delta.x; break;
        case Rail::Top:
        case Rail::Bottom:
            if (std::abs(delta.y)<=kEps) return {};
            t=(plane-start.y)/delta.y; break;
    }
    if (!(t>kEps && t<1.0-kEps)) return {};
    const Vec2 bankCenter=start+delta*t;
    if (!centerOnRailSpan(bankCenter,r,rail)) return {};
    const Vec2 n=railNormal(rail);
    const Vec2 bankContact=bankCenter-n*ballRadius;
    CushionHit hit{rail,bankCenter,bankContact,n,norm(bankCenter-start)};
    if (impactInsidePocketMouth(hit,pockets)) return {};
    const Vec2 incoming=normalized(bankCenter-start);
    const Vec2 outgoing=reflect(incoming,n);
    const Vec2 toward=normalized(destination-bankCenter);
    if (norm2(outgoing)<=kEps || norm2(toward)<=kEps) return {};
    if (dot(outgoing,toward) < 1.0-1e-6) return {};
    return BankRoute{true,rail,bankCenter,bankContact,outgoing};
}

} // namespace goat::physics012
