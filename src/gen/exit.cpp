#include "gen/exit.h"

#include <cmath>
#include <utility>

namespace gen {

ExitConditions::ExitConditions(const std::uint32_t timeoutMs) : timeoutMs_(timeoutMs) {}

void ExitConditions::setTimeout(const std::uint32_t timeoutMs) { timeoutMs_ = timeoutMs; }

bool ExitConditions::hasTimeout() const { return timeoutMs_ > 0; }

void ExitConditions::restartTimer() { startMs_ = pros::millis(); }

void ExitConditions::add(const std::string& name, Check check) { checks_[name] = std::move(check); }

void ExitConditions::remove(const std::string& name) { checks_.erase(name); }

void ExitConditions::clear() { checks_.clear(); }

void ExitConditions::mergeFrom(const ExitConditions& other) {
  for (const auto& [name, fn] : other.checks_) {
    checks_[name] = fn;
  }
  if (!hasTimeout() && other.hasTimeout()) {
    timeoutMs_ = other.timeoutMs_;
  }
}

bool ExitConditions::shouldContinue() const {
  if (timeoutMs_ > 0 && pros::millis() - startMs_ > timeoutMs_) {
    return false;
  }

  for (const auto& [_, check] : checks_) {
    if (check && !check()) return false;
  }
  return true;
}

ExitConditions::Check ExitConditions::errorAbove(std::function<double()> errorSupplier,
                                                 const double minError) {
  return [errorSupplier = std::move(errorSupplier), minError]() {
    return std::fabs(errorSupplier()) > minError;
  };
}

ExitConditions::Check ExitConditions::velocityAbove(std::function<double()> velocitySupplier,
                                                    const double minVelocity) {
  return [velocitySupplier = std::move(velocitySupplier), minVelocity]() {
    return std::fabs(velocitySupplier()) > minVelocity;
  };
}

ExitConditions::Check ExitConditions::halfPlaneNotCrossed(std::function<Pose()> poseSupplier,
                                                          const Pose target,
                                                          const double headingDeg,
                                                          const double tolerance) {
  return [poseSupplier = std::move(poseSupplier), target, headingDeg, tolerance]() {
    const Pose pose = poseSupplier();
    const double thetaRad = deg_to_rad(headingDeg);
    const double lhs = (pose.y - target.y) * -std::cos(thetaRad);
    const double rhs = std::sin(thetaRad) * (pose.x - target.x) + tolerance;
    return lhs >= rhs;
  };
}

}  // namespace gen
