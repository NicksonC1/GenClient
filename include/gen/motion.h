#pragma once

#include <cstdint>

#include "gen/misc.h"
#include "gen/odom.h"
#include "gen/pid.h"
#include "pros/motors.hpp"

namespace gen {

/**
 * Motion helper inspired by the inspo motion algorithms.
 * Provides higher-level moves that pair odometry feedback with PID loops.
 */
class Motion {
 public:
  struct Tuning {
    double kP{0.0};
    double kI{0.0};
    double kD{0.0};
    double headingToleranceDeg{2.0};
    std::int32_t settleTimeMs{250};
    std::int32_t timeoutMs{3000};
    double maxCommand{127.0};
    double minCommand{0.0};
  };

  Motion(pros::MotorGroup& left, pros::MotorGroup& right, Tuning lateral, Tuning angular);

  // Rotate to an absolute field heading (degrees).
  void turnHeading(double targetDeg,
                   double minPower = 0.0,
                   double maxPower = 100.0,
                   std::uint32_t timeoutMs = 3000);

  // Rotate to face a field point.
  void turnPoint(double targetX,
                 double targetY,
                 double minPower = 0.0,
                 double maxPower = 100.0,
                 std::uint32_t timeoutMs = 3000);

  // Drive to a point while correcting heading toward that point.
  void movePoint(double targetX,
                 double targetY,
                 double minPower = 0.0,
                 double maxPower = 100.0,
                 std::uint32_t timeoutMs = 4000,
                 bool settle = true);

  // Drive to a pose with lookahead leads (dLead along heading, gLead lateral to heading).
  void movePose(double targetX,
                double targetY,
                double targetHeadingDeg,
                double dLead,
                double gLead,
                double minPower = 0.0,
                double maxPower = 100.0,
                std::uint32_t timeoutMs = 5000,
                bool settle = true);

  void stop();

 private:
  static double wrapAngleDeg(double angleDeg);
  double clampPower(double value, double maxPower, const Tuning& tuning) const;

  pros::MotorGroup& left_;
  pros::MotorGroup& right_;
  Tuning lateral_;
  Tuning angular_;
  PID lateralPid_;
  PID headingPid_;
};

}  // namespace gen
