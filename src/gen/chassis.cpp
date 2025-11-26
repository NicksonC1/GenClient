#include "gen/chassis.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "pros/rtos.hpp"

namespace gen {

Chassis::Chassis(pros::MotorGroup& left,
                 pros::MotorGroup& right,
                 const Drivetrain& drivetrain,
                 Tuning lateral,
                 Tuning angular)
    : left_(left),
      right_(right),
      drivetrain_(drivetrain),
      lateral_(lateral),
      angular_(angular) {}

Pose Chassis::getPose(const bool radians) const { return gen::getPose(radians); }

double Chassis::clampPower(const double value, const double max_power, const Tuning& tuning) const {
  const double bounded = std::clamp(value, -max_power, max_power);
  return std::clamp(bounded, -tuning.maxCommand, tuning.maxCommand);
}

double Chassis::wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

void Chassis::tank(const int left_power, const int right_power) {
  const int left_cmd = static_cast<int>(std::clamp(left_power, -127, 127));
  const int right_cmd = static_cast<int>(std::clamp(right_power, -127, 127));
  left_.move(left_cmd);
  right_.move(right_cmd);
}

void Chassis::arcade(const int forward, const int turn) { tank(forward + turn, forward - turn); }

void Chassis::stop() {
  left_.move(0);
  right_.move(0);
}

void Chassis::driveDistance(const double distance, const double max_power, ExitConditions* exits) {
  const Pose pose = getPose(true);
  const double target_x = pose.x + distance * std::sin(pose.theta);
  const double target_y = pose.y + distance * std::cos(pose.theta);
  driveToFieldTarget(target_x, target_y, max_power, exits);
}

void Chassis::strafeDistance(const double distance, const double max_power, ExitConditions* exits) {
  const Pose pose = getPose(true);
  const double target_x = pose.x + distance * -std::cos(pose.theta);
  const double target_y = pose.y + distance * std::sin(pose.theta);
  driveToFieldTarget(target_x, target_y, max_power, exits);
}

void Chassis::turnToHeading(const double heading_deg, const double max_power, ExitConditions* exits) {
  const double target_rad = deg_to_rad(heading_deg);
  ExitConditions exitChecks;
  if (exits != nullptr) exitChecks.mergeFrom(*exits);
  exitChecks.restartTimer();
  if (!exitChecks.hasTimeout()) exitChecks.setTimeout(angular_.timeoutMs);
  std::uint32_t settle_start = pros::millis();

  double integral = 0.0;
  double prev_error = 0.0;
  double error_deg = 0.0;

  exitChecks.add("heading-settle", [&]() {
    if (std::abs(error_deg) < angular_.headingToleranceDeg) {
      return pros::millis() - settle_start < static_cast<std::uint32_t>(angular_.settleTimeMs);
    }
    settle_start = pros::millis();
    return true;
  });

  while (true) {
    const Pose pose = getPose(true);
    const double error_rad = wrapAngle(target_rad - pose.theta);
    error_deg = rad_to_deg(error_rad);

    integral += error_deg;
    const double derivative = error_deg - prev_error;
    prev_error = error_deg;

    const double turn_cmd = clampPower(
        error_deg * angular_.kP + integral * angular_.kI + derivative * angular_.kD, max_power,
        angular_);
    tank(static_cast<int>(turn_cmd), static_cast<int>(-turn_cmd));

    if (!exitChecks.shouldContinue()) break;
    pros::delay(10);
  }

  stop();
}

void Chassis::driveToPoint(const double target_x,
                           const double target_y,
                           const double max_power,
                           ExitConditions* exits) {
  driveToFieldTarget(target_x, target_y, max_power, exits);
}

void Chassis::driveToPose(const Pose& target, const double max_power, ExitConditions* exits) {
  driveToFieldTarget(target.x, target.y, max_power, exits);
  turnToHeading(target.theta, max_power, exits);
}

void Chassis::driveToFieldTarget(const double target_x,
                                 const double target_y,
                                 const double max_power,
                                 ExitConditions* exits) {
  const double distance_tolerance =
      drivetrain_.wheelDiameter() > 0.0 ? drivetrain_.wheelDiameter() * 0.1 : 0.5;

  ExitConditions exitChecks;
  if (exits != nullptr) exitChecks.mergeFrom(*exits);
  exitChecks.restartTimer();
  if (!exitChecks.hasTimeout()) exitChecks.setTimeout(lateral_.timeoutMs);
  std::uint32_t settle_start = pros::millis();

  double integral_dist = 0.0;
  double prev_error_dist = 0.0;
  double integral_heading = 0.0;
  double prev_error_heading = 0.0;
  double distance = 0.0;
  double heading_error_deg = 0.0;

  // Default: keep running until the robot crosses the line through the target (half-plane check).
  // This mirrors the inspo behavior and remains even if no custom exits are provided.
  const Pose startPose = getPose(true);
  const double approachHeadingDeg = rad_to_deg(std::atan2(target_x - startPose.x, target_y - startPose.y));
  exitChecks.add("line-cross", ExitConditions::halfPlaneNotCrossed(
                                   [this]() { return getPose(true); },
                                   {target_x, target_y, 0.0},
                                   approachHeadingDeg,
                                   0.0));

  exitChecks.add("pose-settle", [&]() {
    const bool withinDistance = distance < distance_tolerance;
    const bool withinHeading = std::abs(heading_error_deg) < angular_.headingToleranceDeg;
    if (!withinDistance || !withinHeading) {
      settle_start = pros::millis();
      return true;
    }
    return pros::millis() - settle_start < static_cast<std::uint32_t>(lateral_.settleTimeMs);
  });

  while (true) {
    const Pose pose = getPose(true);
    const double dx = target_x - pose.x;
    const double dy = target_y - pose.y;
    distance = std::hypot(dx, dy);

    const double target_heading = std::atan2(dx, dy);
    const double heading_error_rad = wrapAngle(target_heading - pose.theta);
    heading_error_deg = rad_to_deg(heading_error_rad);

    integral_dist += distance;
    const double derivative_dist = distance - prev_error_dist;
    prev_error_dist = distance;
    const double forward_cmd = clampPower(
        distance * lateral_.kP + integral_dist * lateral_.kI + derivative_dist * lateral_.kD,
        max_power, lateral_);

    integral_heading += heading_error_deg;
    const double derivative_heading = heading_error_deg - prev_error_heading;
    prev_error_heading = heading_error_deg;
    const double turn_cmd =
        clampPower(heading_error_deg * angular_.kP + integral_heading * angular_.kI +
                       derivative_heading * angular_.kD,
                   max_power, angular_);

    const double left_cmd = clampPower(forward_cmd + turn_cmd, max_power, lateral_);
    const double right_cmd = clampPower(forward_cmd - turn_cmd, max_power, lateral_);
    tank(static_cast<int>(left_cmd), static_cast<int>(right_cmd));

    if (!exitChecks.shouldContinue()) break;
    pros::delay(10);
  }

  stop();
}

}  // namespace gen
