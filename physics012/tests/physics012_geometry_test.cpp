#include "goat/physics012_geometry.h"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace goat::physics012;

namespace {

bool near(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

std::array<Pocket, 6> standardPockets() {
    return {{
        {{0, 0}, PocketKind::Corner, 12},
        {{100, 0}, PocketKind::Side, 11},
        {{200, 0}, PocketKind::Corner, 12},
        {{0, 100}, PocketKind::Corner, 12},
        {{100, 100}, PocketKind::Side, 11},
        {{200, 100}, PocketKind::Corner, 12},
    }};
}

void testHorizontalReflection() {
    const Vec2 out = reflect({1, 0}, {-1, 0});
    assert(near(out.x, -1));
    assert(near(out.y, 0));
}

void testVerticalReflection() {
    const Vec2 out = reflect({0, 1}, {0, -1});
    assert(near(out.x, 0));
    assert(near(out.y, -1));
}

void testObliqueReflection() {
    const Vec2 out = reflect(normalized({1, 1}), {-1, 0});
    assert(near(out.x, -std::sqrt(0.5)));
    assert(near(out.y,  std::sqrt(0.5)));
}

void testBallRadiusImpact() {
    const Rect table{0, 0, 200, 100};
    auto hit = firstCushionHit({100, 50}, {1, 0}, table, 5);
    assert(hit.has_value());
    assert(hit->rail == Rail::Right);
    assert(near(hit->ballCenterAtImpact.x, 195));
    assert(near(hit->cushionContactPoint.x, 200));
}

void testPocketMouthRejectsRail() {
    const auto pockets = standardPockets();
    CushionHit hit{Rail::Top, {100, 5}, {100, 0}, {0, 1}, 0};
    assert(impactInsidePocketMouth(hit, pockets));

    CushionHit away{Rail::Top, {60, 5}, {60, 0}, {0, 1}, 0};
    assert(!impactInsidePocketMouth(away, pockets));
}

void testObstacleCorridor() {
    std::vector<Ball> balls{{7, {50, 50}, 5}, {8, {50, 75}, 5}};
    assert(segmentBlockedByBalls({{10, 50}, {90, 50}}, balls, 5));
    assert(!segmentBlockedByBalls({{10, 20}, {90, 20}}, balls, 5));
}

void testDegenerateNoCrash() {
    const Rect table{0, 0, 200, 100};
    assert(!firstCushionHit({100, 50}, {0, 0}, table, 5).has_value());
    assert(near(pointSegmentDistance({1, 1}, {0, 0}, {0, 0}), std::sqrt(2.0)));
}

void testOneBankTop() {
    const Rect table{0, 0, 200, 100};
    auto pockets = standardPockets();
    // Bank point lands near x=80, clear of the top side-pocket mouth.
    const auto route = computeOneBankRoute({30, 70}, {130, 70}, Rail::Top, table, 5, pockets);
    assert(route.valid);
    assert(route.rail == Rail::Top);
    assert(near(route.bankCenter.y, 5));
    assert(route.bankCenter.x > 30 && route.bankCenter.x < 130);
}

void testOneBankRejectsPocketMouth() {
    const Rect table{0, 0, 200, 100};
    auto pockets = standardPockets();
    // Symmetric points force the bank near the top side pocket at x=100.
    const auto route = computeOneBankRoute({80, 70}, {120, 70}, Rail::Top, table, 5, pockets);
    assert(!route.valid);
}

void testSymmetryLeftRight() {
    const Rect table{0, 0, 200, 100};
    const auto pockets = standardPockets();
    const auto left = computeOneBankRoute({120, 40}, {120, 60}, Rail::Left, table, 5, pockets);
    const auto right = computeOneBankRoute({80, 40}, {80, 60}, Rail::Right, table, 5, pockets);
    assert(left.valid == right.valid);
    if (left.valid && right.valid) {
        assert(near(left.bankCenter.y, right.bankCenter.y));
        assert(near(left.bankCenter.x + right.bankCenter.x, 200));
    }
}

}  // namespace

int main() {
    testHorizontalReflection();
    testVerticalReflection();
    testObliqueReflection();
    testBallRadiusImpact();
    testPocketMouthRejectsRail();
    testObstacleCorridor();
    testDegenerateNoCrash();
    testOneBankTop();
    testOneBankRejectsPocketMouth();
    testSymmetryLeftRight();
    std::cout << "GOATMOD_PHYSICS_012_GEOMETRY_TEST=PASS\n";
    return 0;
}
