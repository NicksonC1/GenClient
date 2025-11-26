#pragma once

#include "gen/exit.h"
#include "gen/odom.h"
#include "pros/motors.hpp"

namespace gen {

/**
 * Drivetrain helper that pairs the drive motors with the odometry module.
 * Movement helpers run small proportional loops against the global odom pose.
 */
class Chassis {
 public:
  // PID gains bundled with motion limits for a movement axis.
  struct Tuning {
    double kP{0.0};
    double kI{0.0};
    double kD{0.0};
    double headingToleranceDeg{2.0};
    std::int32_t settleTimeMs{250};
    std::int32_t timeoutMs{3000};
    double maxCommand{127.0};
  };

  /**
   * @param left reference to the left drive motor group
   * @param right reference to the right drive motor group
   * @param drivetrain drivetrain geometry used for odometry conversions
   * @param lateral tuning gains and limits for linear motion helper loops
   * @param angular tuning gains and limits for heading/turn helper loops
   */
  Chassis(pros::MotorGroup& left,
          pros::MotorGroup& right,
          const Drivetrain& drivetrain,
          Tuning lateral = {8.0, 0.0, 0.0, 2.0, 250, 4000, 127.0},
          Tuning angular = {2.0, 0.0, 0.0, 2.0, 250, 3000, 127.0});

  Pose getPose(bool radians = false) const;
  void tank(int left_power, int right_power);
  void arcade(int forward, int turn);
  void stop();

  // Sample movement helpers driven by odometry.
  void driveDistance(double distance, double max_power = 100.0, ExitConditions* exits = nullptr);
  void strafeDistance(double distance, double max_power = 100.0, ExitConditions* exits = nullptr);
  void turnToHeading(double heading_deg, double max_power = 80.0, ExitConditions* exits = nullptr);
  void driveToPoint(double target_x, double target_y, double max_power = 100.0, ExitConditions* exits = nullptr);
  // target.theta is interpreted in degrees for user convenience.
  void driveToPose(const Pose& target, double max_power = 100.0, ExitConditions* exits = nullptr);

 private:
  void driveToFieldTarget(double target_x, double target_y, double max_power, ExitConditions* exits);
  double clampPower(double value, double max_power, const Tuning& tuning) const;
  static double wrapAngle(double angle);

  pros::MotorGroup& left_;
  pros::MotorGroup& right_;
  Drivetrain drivetrain_;
  Tuning lateral_;
  Tuning angular_;
};

}  // namespace gen
