#include "gen/misc.h"

namespace gen {

double deg_to_rad(const double deg) { return deg * kPi / 180.0; }
double rad_to_deg(const double rad) { return rad * 180.0 / kPi; }
double ema(const double sample, const double prev, const double alpha) {
  return alpha * prev + (1.0 - alpha) * sample;
}

}  // namespace gen
