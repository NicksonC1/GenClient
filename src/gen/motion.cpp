#include "gen/motion.h"

#include <algorithm>
#include <cmath>

#include "pros/rtos.hpp"

namespace {
constexpr double kPowerScale = 1.0;  // commands already in motor units (0-127)
}

namespace gen {

Motion::Motion(pros::MotorGroup& left,
               pros::MotorGroup& right,
               const Tuning lateral,
               const Tuning angular)
    : left_(left),
      right_(right),
      lateral_(lateral),
      angular_(angular),
      lateralPid_(lateral.kP, lateral.kI, lateral.kD, 0, false),
      headingPid_(angular.kP, angular.kI, angular.kD, 0, false) {}

Pose Motion::getPose(const bool radians) const { return gen::getPose(radians); }

void Motion::tank(const int leftPower, const int rightPower) {
  const int leftCmd = static_cast<int>(std::clamp(leftPower, -127, 127));
  const int rightCmd = static_cast<int>(std::clamp(rightPower, -127, 127));
  left_.move(leftCmd);
  right_.move(rightCmd);
}

void Motion::arcade(const int forward, const int turn) { tank(forward + turn, forward - turn); }

void Motion::stop() {
  left_.move(0);
  right_.move(0);
}

double Motion::wrapAngleDeg(double angleDeg) {
  while (angleDeg > 180.0) angleDeg -= 360.0;
  while (angleDeg < -180.0) angleDeg += 360.0;
  return angleDeg;
}

double Motion::clampPower(const double value, const double maxPower, const Tuning& tuning) const {
  const double bounded = std::clamp(value, -maxPower * kPowerScale, maxPower * kPowerScale);
  const double capped = std::clamp(bounded, -tuning.maxCommand, tuning.maxCommand);
  if (std::abs(capped) < tuning.minCommand) return std::copysign(tuning.minCommand, capped);
  return capped;
}

void Motion::turnHeading(const double targetDeg,
                         const std::uint32_t timeoutMs,
                         const double minPower,
                         const double maxPower) {
  headingPid_.reset();

  const std::uint32_t start = pros::millis();
  std::uint32_t settleStart = start;

  while (pros::millis() - start < timeoutMs) {
    const Pose pose = gen::getPose(true);  // theta in radians
    const double errorDeg = wrapAngleDeg(targetDeg - rad_to_deg(pose.theta));

    double cmd = clampPower(headingPid_.update(errorDeg), maxPower, angular_);
    if (std::abs(cmd) < minPower * kPowerScale) cmd = std::copysign(minPower * kPowerScale, cmd);
    left_.move(static_cast<int>(cmd));
    right_.move(static_cast<int>(-cmd));

    if (std::abs(errorDeg) < angular_.headingToleranceDeg) {
      if (pros::millis() - settleStart > static_cast<std::uint32_t>(angular_.settleTimeMs)) break;
    } else {
      settleStart = pros::millis();
    }

    pros::delay(10);
  }

  stop();
}

void Motion::turnPoint(const double targetX,
                       const double targetY,
                       const std::uint32_t timeoutMs,
                       const double minPower,
                       const double maxPower) {
  const Pose pose = gen::getPose(true);
  const double targetHeading =
      rad_to_deg(std::atan2(targetX - pose.x, targetY - pose.y));  // face the point
  turnHeading(targetHeading, timeoutMs, minPower, maxPower);
}

void Motion::movePoint(const double targetX,
                       const double targetY,
                       const std::uint32_t timeoutMs,
                       const double minPower,
                       const double maxPower,
                       const bool settle,
                       const bool forward) {
  lateralPid_.reset();
  headingPid_.reset();

  const double distanceTolerance = 0.5;  // inches
  const std::uint32_t start = pros::millis();
  std::uint32_t settleStart = start;

  while (pros::millis() - start < timeoutMs) {
    const Pose pose = gen::getPose(true);
    const double dx = targetX - pose.x;
    const double dy = targetY - pose.y;
    const double distance = std::hypot(dx, dy);

    const double targetHeading = std::atan2(dx, dy) + (forward ? 0.0 : kPi);
    const double headingErrorDeg = wrapAngleDeg(rad_to_deg(targetHeading - pose.theta));

    double forwardCmd = clampPower(lateralPid_.update(distance), maxPower, lateral_);
    if (!forward) forwardCmd *= -1;
    double turnCmd = clampPower(headingPid_.update(headingErrorDeg), maxPower, angular_);
    if (std::abs(forwardCmd) < minPower * kPowerScale) forwardCmd = std::copysign(minPower * kPowerScale, forwardCmd);

    const double leftCmd = clampPower(forwardCmd + turnCmd, maxPower, lateral_);
    const double rightCmd = clampPower(forwardCmd - turnCmd, maxPower, lateral_);
    left_.move(static_cast<int>(leftCmd));
    right_.move(static_cast<int>(rightCmd));

    if (distance < distanceTolerance && std::abs(headingErrorDeg) < angular_.headingToleranceDeg) {
      if (!settle || pros::millis() - settleStart > static_cast<std::uint32_t>(lateral_.settleTimeMs)) break;
    } else {
      settleStart = pros::millis();
    }

    pros::delay(10);
  }

  stop();
}

void Motion::moveDistance(const double distance,
                          const std::uint32_t timeoutMs,
                          const bool forward,
                          const double minPower,
                          const double maxPower,
                          const bool settle) {
  const Pose pose = gen::getPose(true);
  const double targetX = pose.x + distance * std::sin(pose.theta);
  const double targetY = pose.y + distance * std::cos(pose.theta);
  movePoint(targetX, targetY, timeoutMs, minPower, maxPower, settle, forward);
}

void Motion::movePose(const double targetX,
                      const double targetY,
                      const double targetHeadingDeg,
                      const double dLead,
                      const double gLead,
                      const std::uint32_t timeoutMs,
                      const bool forward,
                       const double minPower,
                       const double maxPower,
                       const bool settle) {
  // Lead the target to reduce corner-cutting (similar to inspo: dLead forward, gLead lateral).
  const double targetHeadingRad = deg_to_rad(targetHeadingDeg);
  const double leadX = targetX + dLead * std::sin(targetHeadingRad) - gLead * std::cos(targetHeadingRad);
  const double leadY = targetY + dLead * std::cos(targetHeadingRad) + gLead * std::sin(targetHeadingRad);

  lateralPid_.reset();
  headingPid_.reset();

  const double distanceTolerance = 0.5;  // inches
  const std::uint32_t start = pros::millis();
  std::uint32_t settleStart = start;

  while (pros::millis() - start < timeoutMs) {
    const Pose pose = gen::getPose(true);
    const double dx = leadX - pose.x;
    const double dy = leadY - pose.y;
    const double distance = std::hypot(dx, dy);

    const double desiredHeading = std::atan2(dx, dy) + (forward ? 0.0 : kPi);
    const double headingErrorDeg =
        wrapAngleDeg(rad_to_deg(desiredHeading - pose.theta));  // point toward the lead
    const double facingErrorDeg =
        wrapAngleDeg(targetHeadingDeg - rad_to_deg(pose.theta));  // align to final heading

    double forwardCmd = clampPower(lateralPid_.update(distance), maxPower, lateral_);
    // Blend heading correction: chase the path, bias toward final orientation as we near target.
    const double headingBlend = std::clamp(distance / (distance + 1.0), 0.0, 1.0);
    const double combinedHeadingError = headingBlend * headingErrorDeg + (1.0 - headingBlend) * facingErrorDeg;
    double turnCmd = clampPower(headingPid_.update(combinedHeadingError), maxPower, angular_);
    if (!forward) forwardCmd *= -1;
    if (std::abs(forwardCmd) < minPower * kPowerScale) forwardCmd = std::copysign(minPower * kPowerScale, forwardCmd);

    const double leftCmd = clampPower(forwardCmd + turnCmd, maxPower, lateral_);
    const double rightCmd = clampPower(forwardCmd - turnCmd, maxPower, lateral_);
    left_.move(static_cast<int>(leftCmd));
    right_.move(static_cast<int>(rightCmd));

    const bool atPosition = distance < distanceTolerance;
    const bool atHeading = std::abs(facingErrorDeg) < angular_.headingToleranceDeg;

    if (atPosition && atHeading) {
      if (!settle || pros::millis() - settleStart > static_cast<std::uint32_t>(lateral_.settleTimeMs)) break;
    } else {
      settleStart = pros::millis();
    }

    pros::delay(10);
  }

  stop();
}

}  // namespace gen
