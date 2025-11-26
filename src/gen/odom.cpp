#include "gen/odom.h"

#include <cmath>
#include <vector>

#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace {
constexpr double kDt = 0.01;  // 10ms loop
constexpr double kEpsilon = 1e-6;
constexpr double kMmToInches = 1.0 / 25.4;
}

namespace gen {
namespace {
pros::Task* trackingTask = nullptr;
OdomSensors odomSensors{};
Drivetrain drive{};
Pose odomPose{};
Pose odomSpeed{};
Pose odomLocalSpeed{};
std::vector<DistanceResetSensor> distanceResetSensors{};
double distanceFieldSize = 144.0;

double prevVertical = 0.0;
double prevVertical1 = 0.0;
double prevVertical2 = 0.0;
double prevHorizontal = 0.0;
double prevHorizontal1 = 0.0;
double prevHorizontal2 = 0.0;
double prevImu = 0.0;
}  // namespace

Drivetrain::Drivetrain(pros::MotorGroup* left,
                       pros::MotorGroup* right,
                       const double gear_ratio,
                       const double wheel_diameter,
                       const double track_width,
                       const double wheelbase)
    : left_(left),
      right_(right),
      gear_ratio_(gear_ratio),
      wheel_diameter_(wheel_diameter),
      track_width_(track_width),
      wheelbase_(wheelbase) {}

double Drivetrain::distanceForGroup(pros::MotorGroup* group) const {
  if (!group || gear_ratio_ == 0.0 || wheel_diameter_ == 0.0) return 0.0;
  const std::vector<double> positions = group->get_position_all();  // degrees
  if (positions.empty()) return 0.0;
  double avg_deg = 0.0;
  for (const double value : positions) avg_deg += value;
  avg_deg /= positions.size();
  const double wheel_rotations = (avg_deg / 360.0) / gear_ratio_;
  return wheel_rotations * wheel_diameter_ * kPi;
}

double Drivetrain::leftDistance() const { return distanceForGroup(left_); }
double Drivetrain::rightDistance() const { return distanceForGroup(right_); }

void Drivetrain::reset() const {
  if (left_ != nullptr) left_->tare_position();
  if (right_ != nullptr) right_->tare_position();
}

TrackingWheel::TrackingWheel(pros::Rotation* rotation,
                             const double diameter,
                             const double offset,
                             const double gear_ratio,
                             const bool reversed)
    : source_(Source::Rotation),
      rotation_(rotation),
      diameter_(diameter),
      gear_ratio_(gear_ratio),
      offset_(offset),
      reversed_(reversed) {}

TrackingWheel::TrackingWheel(const Source drive_source,
                             const double offset,
                             const bool reversed)
    : source_(drive_source),
      rotation_(nullptr),
      diameter_(0.0),
      gear_ratio_(1.0),
      offset_(offset),
      reversed_(reversed) {}

double TrackingWheel::getDistanceTraveled() const {
  double distance = 0.0;
  switch (source_) {
    case Source::Rotation: {
      if (rotation_ == nullptr) return 0.0;
      const double degrees = rotation_->get_position();  // degrees
      const double wheel_rotations =
          (degrees / 360.0) / (gear_ratio_ == 0.0 ? 1.0 : gear_ratio_);
      distance = wheel_rotations * diameter_ * kPi;
      break;
    }
    case Source::DriveLeft:
      distance = drive.leftDistance();
      break;
    case Source::DriveRight:
      distance = drive.rightDistance();
      break;
  }
  return reversed_ ? -distance : distance;
}

void TrackingWheel::reset() const {
  switch (source_) {
    case Source::Rotation:
      if (rotation_ != nullptr) rotation_->reset_position();
      break;
    case Source::DriveLeft:
    case Source::DriveRight:
      drive.reset();
      break;
  }
}

void setSensors(const OdomSensors sensors, const Drivetrain drivetrain) {
  odomSensors = sensors;
  drive = drivetrain;

  if (odomSensors.vertical1 != nullptr) odomSensors.vertical1->reset();
  if (odomSensors.vertical2 != nullptr) odomSensors.vertical2->reset();
  if (odomSensors.horizontal1 != nullptr) odomSensors.horizontal1->reset();
  if (odomSensors.horizontal2 != nullptr) odomSensors.horizontal2->reset();

  prevVertical1 = odomSensors.vertical1 != nullptr ? odomSensors.vertical1->getDistanceTraveled() : 0.0;
  prevVertical2 = odomSensors.vertical2 != nullptr ? odomSensors.vertical2->getDistanceTraveled() : 0.0;
  prevHorizontal1 =
      odomSensors.horizontal1 != nullptr ? odomSensors.horizontal1->getDistanceTraveled() : 0.0;
  prevHorizontal2 =
      odomSensors.horizontal2 != nullptr ? odomSensors.horizontal2->getDistanceTraveled() : 0.0;
  prevVertical = prevVertical1;
  prevHorizontal = prevHorizontal1;
  prevImu = odomSensors.imu != nullptr ? deg_to_rad(odomSensors.imu->get_rotation()) : 0.0;
}

void setDistanceResetSensors(const std::vector<DistanceResetSensor>& sensors,
                             const double fieldSizeInches) {
  distanceResetSensors = sensors;
  distanceFieldSize = fieldSizeInches;
}

bool resetFromDistanceSensors() {
  if (distanceResetSensors.empty()) return false;
  const double theta =
      odomSensors.imu != nullptr ? deg_to_rad(odomSensors.imu->get_heading()) : odomPose.theta;
  const double halfField = distanceFieldSize / 2.0;

  const DistanceResetSensor* front = nullptr;
  const DistanceResetSensor* back = nullptr;
  const DistanceResetSensor* left = nullptr;
  const DistanceResetSensor* right = nullptr;

  for (const auto& sensor : distanceResetSensors) {
    switch (sensor.side) {
      case DistanceResetSensor::Side::Front:
        if (front == nullptr) front = &sensor;
        break;
      case DistanceResetSensor::Side::Back:
        if (back == nullptr) back = &sensor;
        break;
      case DistanceResetSensor::Side::Left:
        if (left == nullptr) left = &sensor;
        break;
      case DistanceResetSensor::Side::Right:
        if (right == nullptr) right = &sensor;
        break;
    }
  }

  double x = odomPose.x;
  double y = odomPose.y;
  bool updatedX = false;
  bool updatedY = false;

  auto readInches = [](pros::Distance* sensor) -> double {
    if (sensor == nullptr) return 0.0;
    return sensor->get_distance() * kMmToInches;
  };

  if (left != nullptr && left->sensor != nullptr) {
    const double dist = readInches(left->sensor);
    const double rotX = left->xOffset * std::cos(theta) + left->yOffset * std::sin(theta);
    x = (dist - halfField) - rotX;
    updatedX = true;
  } else if (right != nullptr && right->sensor != nullptr) {
    const double dist = readInches(right->sensor);
    const double rotX = right->xOffset * std::cos(theta) + right->yOffset * std::sin(theta);
    x = (halfField - dist) - rotX;
    updatedX = true;
  }

  if (front != nullptr && front->sensor != nullptr) {
    const double dist = readInches(front->sensor);
    const double rotY = front->yOffset * std::cos(theta) - front->xOffset * std::sin(theta);
    y = (halfField - dist) - rotY;
    updatedY = true;
  } else if (back != nullptr && back->sensor != nullptr) {
    const double dist = readInches(back->sensor);
    const double rotY = back->yOffset * std::cos(theta) - back->xOffset * std::sin(theta);
    y = (dist - halfField) + rotY;
    updatedY = true;
  }

  if (!updatedX && !updatedY) return false;

  setPose({x, y, theta}, true);
  return true;
}

Pose getPose(const bool radians) {
  if (radians) return odomPose;
  return {odomPose.x, odomPose.y, rad_to_deg(odomPose.theta)};
}

void setPose(const Pose pose, const bool radians) {
  if (radians) {
    odomPose = pose;
  } else {
    odomPose = {pose.x, pose.y, deg_to_rad(pose.theta)};
  }
}

Pose getSpeed(const bool radians) {
  if (radians) return odomSpeed;
  return {odomSpeed.x, odomSpeed.y, rad_to_deg(odomSpeed.theta)};
}

Pose getLocalSpeed(const bool radians) {
  if (radians) return odomLocalSpeed;
  return {odomLocalSpeed.x, odomLocalSpeed.y, rad_to_deg(odomLocalSpeed.theta)};
}

Pose estimatePose(const float time, const bool radians) {
  const Pose curPose = getPose(true);
  const Pose localSpeed = getLocalSpeed(true);
  const Pose deltaLocalPose = localSpeed * time;

  const double avgHeading = curPose.theta + deltaLocalPose.theta / 2.0;
  Pose futurePose = curPose;
  futurePose.x += deltaLocalPose.y * std::sin(avgHeading);
  futurePose.y += deltaLocalPose.y * std::cos(avgHeading);
  futurePose.x += deltaLocalPose.x * -std::cos(avgHeading);
  futurePose.y += deltaLocalPose.x * std::sin(avgHeading);
  if (!radians) futurePose.theta = rad_to_deg(futurePose.theta);

  return futurePose;
}

void update() {
  // Fetch current sensor values.
  const double vertical1Raw =
      odomSensors.vertical1 != nullptr ? odomSensors.vertical1->getDistanceTraveled() : 0.0;
  const double vertical2Raw =
      odomSensors.vertical2 != nullptr ? odomSensors.vertical2->getDistanceTraveled() : 0.0;
  const double horizontal1Raw =
      odomSensors.horizontal1 != nullptr ? odomSensors.horizontal1->getDistanceTraveled() : 0.0;
  const double horizontal2Raw =
      odomSensors.horizontal2 != nullptr ? odomSensors.horizontal2->getDistanceTraveled() : 0.0;
  const double imuRaw = odomSensors.imu != nullptr ? deg_to_rad(odomSensors.imu->get_rotation()) : 0.0;

  // Compute deltas.
  const double deltaVertical1 = vertical1Raw - prevVertical1;
  const double deltaVertical2 = vertical2Raw - prevVertical2;
  const double deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
  const double deltaHorizontal2 = horizontal2Raw - prevHorizontal2;
  const double deltaImu = imuRaw - prevImu;

  prevVertical1 = vertical1Raw;
  prevVertical2 = vertical2Raw;
  prevHorizontal1 = horizontal1Raw;
  prevHorizontal2 = horizontal2Raw;
  prevImu = imuRaw;

  // Heading estimation priority: horizontal wheels -> non-drive vertical wheels -> IMU -> fallback.
  double heading = odomPose.theta;
  if (odomSensors.horizontal1 != nullptr && odomSensors.horizontal2 != nullptr) {
    const double offsetDiff =
        odomSensors.horizontal1->getOffset() - odomSensors.horizontal2->getOffset();
    if (std::abs(offsetDiff) > kEpsilon) {
      heading -= (deltaHorizontal1 - deltaHorizontal2) / offsetDiff;
    }
  } else if (odomSensors.vertical1 != nullptr && odomSensors.vertical2 != nullptr &&
             !odomSensors.vertical1->isSubstitute() && !odomSensors.vertical2->isSubstitute()) {
    const double offsetDiff = odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset();
    if (std::abs(offsetDiff) > kEpsilon) {
      heading -= (deltaVertical1 - deltaVertical2) / offsetDiff;
    }
  } else if (odomSensors.imu != nullptr) {
    heading += deltaImu;
  } else if (odomSensors.vertical1 != nullptr && odomSensors.vertical2 != nullptr) {
    const double offsetDiff = odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset();
    if (std::abs(offsetDiff) > kEpsilon) {
      heading -= (deltaVertical1 - deltaVertical2) / offsetDiff;
    }
  }

  const double deltaHeading = heading - odomPose.theta;
  const double avgHeading = odomPose.theta + deltaHeading / 2.0;

  // Choose translation trackers (favor non-drive wheels).
  TrackingWheel* verticalWheel = nullptr;
  TrackingWheel* horizontalWheel = nullptr;

  if (odomSensors.vertical1 != nullptr && !odomSensors.vertical1->isSubstitute()) {
    verticalWheel = odomSensors.vertical1;
  } else if (odomSensors.vertical2 != nullptr && !odomSensors.vertical2->isSubstitute()) {
    verticalWheel = odomSensors.vertical2;
  } else if (odomSensors.vertical1 != nullptr) {
    verticalWheel = odomSensors.vertical1;
  } else if (odomSensors.vertical2 != nullptr) {
    verticalWheel = odomSensors.vertical2;
  }

  if (odomSensors.horizontal1 != nullptr) {
    horizontalWheel = odomSensors.horizontal1;
  } else if (odomSensors.horizontal2 != nullptr) {
    horizontalWheel = odomSensors.horizontal2;
  }

  const double rawVertical = verticalWheel != nullptr ? verticalWheel->getDistanceTraveled() : 0.0;
  const double rawHorizontal =
      horizontalWheel != nullptr ? horizontalWheel->getDistanceTraveled() : 0.0;
  const double verticalOffset = verticalWheel != nullptr ? verticalWheel->getOffset() : 0.0;
  const double horizontalOffset = horizontalWheel != nullptr ? horizontalWheel->getOffset() : 0.0;

  const double deltaY = rawVertical - prevVertical;
  const double deltaX = rawHorizontal - prevHorizontal;
  prevVertical = rawVertical;
  prevHorizontal = rawHorizontal;

  double localX = 0.0;
  double localY = 0.0;
  if (std::abs(deltaHeading) < kEpsilon) {
    localX = deltaX;
    localY = deltaY;
  } else {
    const double arc = 2.0 * std::sin(deltaHeading / 2.0);
    localX = arc * (deltaX / deltaHeading + horizontalOffset);
    localY = arc * (deltaY / deltaHeading + verticalOffset);
  }

  const Pose prevPose = odomPose;

  odomPose.x += localY * std::sin(avgHeading);
  odomPose.y += localY * std::cos(avgHeading);
  odomPose.x += localX * -std::cos(avgHeading);
  odomPose.y += localX * std::sin(avgHeading);
  odomPose.theta = heading;

  odomSpeed.x = ema((odomPose.x - prevPose.x) / kDt, odomSpeed.x, 0.95);
  odomSpeed.y = ema((odomPose.y - prevPose.y) / kDt, odomSpeed.y, 0.95);
  odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / kDt, odomSpeed.theta, 0.95);

  odomLocalSpeed.x = ema(localX / kDt, odomLocalSpeed.x, 0.95);
  odomLocalSpeed.y = ema(localY / kDt, odomLocalSpeed.y, 0.95);
  odomLocalSpeed.theta = ema(deltaHeading / kDt, odomLocalSpeed.theta, 0.95);
}

void init() {
  if (trackingTask == nullptr) {
    trackingTask = new pros::Task([] {
      while (true) {
        update();
        pros::delay(10);
      }
    },
    "odom-tracker");
  }
}

}  // namespace gen
