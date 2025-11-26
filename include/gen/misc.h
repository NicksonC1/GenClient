#pragma once

#include <cmath>

namespace gen {

constexpr double kPi = 3.14159265358979323846;

constexpr int getSign(auto value) { return value < 0 ? -1 : 1; }

double deg_to_rad(double deg);
double rad_to_deg(double rad);
double ema(double sample, double prev, double alpha);

}  // namespace gen
