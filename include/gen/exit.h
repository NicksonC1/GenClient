#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>

#include "gen/misc.h"
#include "gen/odom.h"
#include "pros/rtos.hpp"

namespace gen {

/**
 * Exit-condition helper inspired by the inspo project.
 * Each check returns true while the motion loop should continue.
 * When any check returns false (or the timeout elapses), the loop exits.
 */
class ExitConditions {
 public:
  using Check = std::function<bool()>;

  ExitConditions() = default;
  explicit ExitConditions(std::uint32_t timeoutMs);

  void setTimeout(std::uint32_t timeoutMs);
  bool hasTimeout() const;
  void restartTimer();

  void add(const std::string& name, Check check);
  void remove(const std::string& name);
  void clear();
  void mergeFrom(const ExitConditions& other);

  bool shouldContinue() const;

  // Convenience factories mirroring the inspo Exit helpers.
  static Check errorAbove(std::function<double()> errorSupplier, double minError);
  static Check velocityAbove(std::function<double()> velocitySupplier, double minVelocity);
  static Check halfPlaneNotCrossed(std::function<Pose()> poseSupplier,
                                   Pose target,
                                   double headingDeg,
                                   double tolerance);

 private:
  std::uint32_t startMs_{pros::millis()};
  std::uint32_t timeoutMs_{0};
  std::map<std::string, Check> checks_;
};

}  // namespace gen
