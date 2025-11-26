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
    double maxCommand{127.0};
    double minCommand{0.0};
  };

  /**
   * @param left motor group to command (left side)
   * @param right motor group to command (right side)
   * @param lateral PID limits and tolerances for translation
   * @param angular PID limits and tolerances for rotation
   */
  Motion(pros::MotorGroup& left, pros::MotorGroup& right, Tuning lateral, Tuning angular);

  // Low-level helpers.
  Pose getPose(bool radians = false) const;
  void tank(int leftPower, int rightPower);
  void arcade(int forward, int turn);
  void stop();

  // Rotate to an absolute field heading (degrees).
  /**
   * @param targetDeg absolute field heading in degrees
   * @param timeoutMs maximum duration before abort
   * @param minPower minimum absolute power to overcome static friction
   * @param maxPower cap in motor units (0-127)
   */
  void turnHeading(double targetDeg,
                   std::uint32_t timeoutMs = 3000,
                   double minPower = 0.0,
                   double maxPower = 127.0);

  // Rotate to face a field point.
  /**
   * @param targetX field X in inches
   * @param targetY field Y in inches
   * @param timeoutMs maximum duration before abort
   * @param minPower minimum absolute power to overcome static friction
   * @param maxPower cap in motor units (0-127)
   */
  void turnPoint(double targetX,
                 double targetY,
                 std::uint32_t timeoutMs = 3000,
                 double minPower = 0.0,
                 double maxPower = 127.0);

  // Drive to a point while correcting heading toward that point.
  /**
   * @param targetX field X in inches
   * @param targetY field Y in inches
   * @param timeoutMs maximum duration before abort
   * @param minPower minimum absolute power to overcome static friction
   * @param maxPower cap in motor units (0-127)
   * @param settle require dwell in tolerance before exit
   * @param forward when false, drive backward toward the target
   */
  void movePoint(double targetX,
                 double targetY,
                 std::uint32_t timeoutMs = 4000,
                 double minPower = 0.0,
                 double maxPower = 127.0,
                 bool settle = true,
                 bool forward = true);

  // Drive a straight-line distance along the current heading.
  /**
   * @param distance inches along current heading (positive forward)
   * @param timeoutMs maximum duration before abort
   * @param forward when false, drive backward toward the target
   * @param minPower minimum absolute power to overcome static friction
   * @param maxPower cap in motor units (0-127)
   * @param settle require dwell in tolerance before exit
   */
  void moveDistance(double distance,
                    std::uint32_t timeoutMs = 4000,
                    bool forward = true,
                    double minPower = 0.0,
                    double maxPower = 127.0,
                    bool settle = true);

  // Drive to a pose with lookahead leads (dLead along heading, gLead lateral to heading).
  /**
   * @param targetX field X in inches
   * @param targetY field Y in inches
   * @param targetHeadingDeg desired final heading (degrees)
   * @param dLead forward lead distance along heading
   * @param gLead lateral lead distance perpendicular to heading
   * @param timeoutMs maximum duration before abort
   * @param forward when false, drive backward toward the target
   * @param minPower minimum absolute power to overcome static friction
   * @param maxPower cap in motor units (0-127)
   * @param settle require dwell in tolerance before exit
   */
  void movePose(double targetX,
                double targetY,
                double targetHeadingDeg,
                double dLead,
                double gLead,
                std::uint32_t timeoutMs = 5000,
                bool forward = true,
                double minPower = 0.0,
                double maxPower = 127.0,
                bool settle = true);

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
