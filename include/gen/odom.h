#pragma once
#include <cstdint>
#include <vector>
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

namespace gen {

constexpr double kPi = 3.14159265358979323846;

struct Pose {
  double x{0.0};
  double y{0.0};
  double theta{0.0};  // radians

  Pose() = default;
  Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}

  Pose operator*(double scalar) const { return {x * scalar, y * scalar, theta * scalar}; }
};

double deg_to_rad(double deg);
double rad_to_deg(double rad);
double ema(double sample, double prev, double alpha);

class Drivetrain {
 public:
  Drivetrain(pros::MotorGroup* left = nullptr,
             pros::MotorGroup* right = nullptr,
             double gear_ratio = 0.0,
             double wheel_diameter = 0.0,
             double track_width = 0.0,
             double wheelbase = 0.0);

  double leftDistance() const;
  double rightDistance() const;
  void reset() const;

  pros::MotorGroup* left() const { return left_; }
  pros::MotorGroup* right() const { return right_; }
  double gearRatio() const { return gear_ratio_; }
  double wheelDiameter() const { return wheel_diameter_; }
  double trackWidth() const { return track_width_; }
  double wheelbase() const { return wheelbase_; }

 private:
  double distanceForGroup(pros::MotorGroup* group) const;

  pros::MotorGroup* left_;
  pros::MotorGroup* right_;
  double gear_ratio_;
  double wheel_diameter_;
  double track_width_;
  double wheelbase_;
};

class TrackingWheel {
 public:
  enum class Source { Rotation, DriveLeft, DriveRight };

  TrackingWheel(pros::Rotation* rotation,
                double diameter,
                double offset,
                double gear_ratio = 1.0,
                bool reversed = false);

  // Use drive motors as a substitute tracking source.
  TrackingWheel(Source drive_source, double offset, bool reversed = false);

  double getDistanceTraveled() const;
  double getOffset() const { return offset_; }
  bool isSubstitute() const { return source_ != Source::Rotation; }
  void reset() const;

 private:
  Source source_;
  pros::Rotation* rotation_;
  double diameter_;
  double gear_ratio_;
  double offset_;
  bool reversed_;
};

struct OdomSensors {
  TrackingWheel* vertical1{nullptr};
  TrackingWheel* vertical2{nullptr};
  TrackingWheel* horizontal1{nullptr};
  TrackingWheel* horizontal2{nullptr};
  pros::Imu* imu{nullptr};
};

struct DistanceResetSensor {
  enum class Side { Front, Back, Left, Right };
  pros::Distance* sensor{nullptr};
  Side side{Side::Front};
  double xOffset{0.0};  // + right
  double yOffset{0.0};  // + forward
};

void setSensors(OdomSensors sensors, Drivetrain drivetrain);
Pose getPose(bool radians = false);
void setPose(Pose pose, bool radians = false);
Pose getSpeed(bool radians = false);
Pose getLocalSpeed(bool radians = false);
Pose estimatePose(float time, bool radians = false);
void setDistanceResetSensors(const std::vector<DistanceResetSensor>& sensors,
                             double fieldSizeInches = 144.0);
bool resetFromDistanceSensors();
void update();
void init();

}  // namespace gen
