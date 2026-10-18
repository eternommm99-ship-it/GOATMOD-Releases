#ifdef NDEBUG
#error "Geometry tests require assertions enabled (NDEBUG must be unset)"
#endif
#include "goat/physics012_geometry.h"
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
using namespace goat::physics012;
namespace {
bool near(double a,double b,double e=1e-6){return std::abs(a-b)<=e;}
std::array<Pocket,6> pockets(){return {{{{0,0},PocketKind::Corner,12},{{100,0},PocketKind::Side,11},{{200,0},PocketKind::Corner,12},{{0,100},PocketKind::Corner,12},{{100,100},PocketKind::Side,11},{{200,100},PocketKind::Corner,12}}};}
void reflections(){auto a=reflect({1,0},{-1,0});assert(near(a.x,-1)&&near(a.y,0));auto b=reflect({0,1},{0,-1});assert(near(b.x,0)&&near(b.y,-1));auto c=reflect(normalized({1,1}),{-1,0});assert(near(c.x,-std::sqrt(.5))&&near(c.y,std::sqrt(.5)));}
void radiusImpact(){Rect t{0,0,200,100};auto h=firstCushionHit({100,50},{1,0},t,5);assert(h&&h->rail==Rail::Right);assert(near(h->ballCenterAtImpact.x,195));assert(near(h->cushionContactPoint.x,200));}
void mouth(){auto p=pockets();CushionHit h{Rail::Top,{100,5},{100,0},{0,1},0};assert(impactInsidePocketMouth(h,p));CushionHit a{Rail::Top,{60,5},{60,0},{0,1},0};assert(!impactInsidePocketMouth(a,p));}
void obstacles(){std::vector<Ball>b{{7,{50,50},5},{8,{50,75},5}};assert(segmentBlockedByBalls({{10,50},{90,50}},b,5));assert(!segmentBlockedByBalls({{10,20},{90,20}},b,5));}
void degenerate(){Rect t{0,0,200,100};assert(!firstCushionHit({100,50},{0,0},t,5));assert(near(pointSegmentDistance({1,1},{0,0},{0,0}),std::sqrt(2.0)));}
void bank(){Rect t{0,0,200,100};auto p=pockets();auto r=computeOneBankRoute({30,70},{130,70},Rail::Top,t,5,p);assert(r.valid&&r.rail==Rail::Top);assert(near(r.bankCenter.y,5));auto bad=computeOneBankRoute({80,70},{120,70},Rail::Top,t,5,p);assert(!bad.valid);}
}
int main(){reflections();radiusImpact();mouth();obstacles();degenerate();bank();std::cout<<"GOATMOD_PHYSICS_012_GEOMETRY_TEST=PASS\n";}
