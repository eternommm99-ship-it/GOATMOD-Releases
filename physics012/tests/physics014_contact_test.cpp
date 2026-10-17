#include "goat/physics014_contact.h"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace goat::physics014;
using goat::physics012::Vec2;

namespace {

bool near(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

void testHeadOnTransfer() {
    ContactInput in;
    in.incomingCueDir = {1, 0};
    in.contactNormal = {1, 0};
    in.cueSpeed = 100.0;
    auto r = solveEqualBallContact(in);
    assert(r.valid);
    assert(near(r.objectDir.x, 1.0));
    assert(near(r.objectDir.y, 0.0));
    assert(near(r.objectSpeed, 100.0));
    assert(near(r.cueSpeed, 0.0));
}

void testCutRetainsTangentialCueMotion() {
    ContactInput in;
    in.incomingCueDir = {1, 1};
    in.contactNormal = {1, 0};
    in.cueSpeed = 100.0;
    auto r = solveEqualBallContact(in);
    assert(r.valid);
    assert(r.objectSpeed > 70.0 && r.objectSpeed < 71.0);
    assert(r.cueSpeed > 70.0 && r.cueSpeed < 71.0);
    assert(near(r.cueDir.x, 0.0, 1e-6));
    assert(near(r.cueDir.y, 1.0, 1e-6));
}

void testSideThrowTuning() {
    ContactInput in;
    in.incomingCueDir = {1, 1};
    in.contactNormal = {1, 0};
    in.cueSpeed = 100.0;
    in.sideSpin = 1.0;

    ContactTuning tuning;
    tuning.sideThrowRadians = 0.1;
    auto r = solveEqualBallContact(in, tuning);
    assert(r.valid);
    assert(r.appliedThrowRadians > 0.0);
    assert(r.objectDir.y > 0.0);
}

void testTopBackSpinGain() {
    ContactInput in;
    in.incomingCueDir = {1, 1};
    in.contactNormal = {1, 0};
    in.cueSpeed = 100.0;

    ContactTuning tuning;
    tuning.followGain = 0.5;

    in.topSpin = 1.0;
    auto follow = solveEqualBallContact(in, tuning);
    in.topSpin = -1.0;
    auto draw = solveEqualBallContact(in, tuning);
    assert(follow.valid && draw.valid);
    assert(follow.cueSpeed > draw.cueSpeed);
}

void testRejectBadApproach() {
    ContactInput in;
    in.incomingCueDir = {-1, 0};
    in.contactNormal = {1, 0};
    in.cueSpeed = 100.0;
    assert(!solveEqualBallContact(in).valid);
}

}  // namespace

int main() {
    testHeadOnTransfer();
    testCutRetainsTangentialCueMotion();
    testSideThrowTuning();
    testTopBackSpinGain();
    testRejectBadApproach();
    std::cout << "GOATMOD_PHYSICS_014_CONTACT_TEST=PASS\n";
    return 0;
}
