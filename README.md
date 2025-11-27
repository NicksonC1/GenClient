<img src="images/genclient.png">

## Features
- controller wrapper with tank/arcade mixes, deadband, and optional drive curves
- motor + piston wrappers (grouped pistons, rpm/gear-aware motor moves, IMU scaling)
- odom core that blends tracking wheels, drive substitutions, and distance-sensor wall resets
- motion layer built on odom: turn heading/point, drive distance/point/pose with PID tuning
- tiny PID + math utilities plus exit-condition combinators for loop control
- auton registry stub (`include/gen/auton.h`) ready for your routines

## Usage
To use genClient, include the headers you need (e.g. `#include "gen/motion.h"`) and initialize odom once with `setSensors(...)` + `init()`. Most helpers expect inches for linear values and degrees for user-facing angles.

<font size = 6>**Hardware abstractions**</font>
---
**motors**
- `gen::MotorGroup` extends `pros::MotorGroup` and remembers motor rpm + external gear ratio.
- helpers:
  - `move_percent(double percent)` clamps [-1, 1] -> +/-127 command
  - `move_rpm(double rpm)` commands motor rpm directly
  - `move_wheel_rpm(double wheel_rpm)` converts via stored gear ratio
  - accessors: `motor_rpm()`, `gear_ratio()`, `wheel_rpm()`

example:
```cpp
gen::MotorGroup left({-11, -12, -13}, 200.0, 1.333); // ports, motor rpm, gear ratio
left.move_percent(0.8);
left.move_rpm(150);
left.move_wheel_rpm(280);
```

**pistons**
- `gen::Piston` wraps `pros::adi::DigitalOut` with cached state and a name tag.
- `gen::PistonGroup` collects named pistons; you can enable/disable individual members.
- helpers: `on()`, `off()`, `toggle()`, `state()`, `enable(name)`, `disable(name)`.

example:
```cpp
gen::Piston wing('A', false, "wing");
gen::PistonGroup wings({{"wing", &wing}});
wings.toggle(); // flips all enabled pistons
```

**imu**
- `gen::CustomIMU` subclasses `pros::IMU` and scales `get_rotation()` by a supplied scalar (useful for calibration quirks).

<font size = 6>**Controller**</font>
---
`gen::Controller` wraps a `pros::Controller` and offers quick drive mixes.

- drive modes: `Tank2Stick`, `Arcade1Stick`, `Arcade2Stick`
- drive helpers: `drive_values()` returns `{left, right}` based on the configured mode
- button helpers: `holding(Button)`, `pressed(Button)`
- tuning: `deadband(int)`, `curve(double amount, bool enabled)`

example teleop loop:
```cpp
gen::Controller driver(gen::Controller::DriveMode::Arcade2Stick, 3, 10.0, true);
while (true) {
  const auto [l, r] = driver.drive_values();
  leftMotors.move(l);
  rightMotors.move(r);
  if (driver.pressed(gen::Controller::Button::A)) wings.toggle();
  pros::delay(10);
}
```

<font size = 6>**Odometry**</font>
---
Odom is modular: mix tracking wheels, substitute drive encoders, and optionally add distance sensors for wall-based resets. Linear units are inches; stored heading is radians internally.

- `gen::Drivetrain` tracks drive groups + geometry (`gear_ratio`, `wheel_diameter`, `track_width`, `wheelbase`). Helpers: `leftDistance()`, `rightDistance()`, `reset()`.
- `gen::TrackingWheel` from a `pros::Rotation` + diameter + lateral offset; can also substitute `DriveLeft` / `DriveRight`.
- `gen::OdomSensors` bundle: up to 2 vertical, 2 horizontal trackers + optional IMU.
- lifecycle:
  - `setSensors(OdomSensors, Drivetrain)`
  - `setDistanceResetSensors(std::vector<DistanceResetSensor>, fieldSizeInches)` (optional)
  - `init()` spins the 10ms tracking task
- accessors:
  - `getPose(bool radians = false)` / `setPose(...)`
  - `getSpeed()` (global) and `getLocalSpeed()` (robot frame)
  - `estimatePose(float timeAheadSeconds)` for basic lookahead
  - `resetFromDistanceSensors()` to snap pose using wall distances when configured

example setup (mirrors `src/main.cpp`):
```cpp
gen::TrackingWheel vL(&verticalLeft, 2.75, 3.5);
gen::TrackingWheel vR(&verticalRight, 2.75, -3.5, 1.0, true);
gen::TrackingWheel h(&horizontal, 2.75, -2.5);
gen::OdomSensors sensors{&vL, &vR, &h, nullptr, &imu};
gen::Drivetrain drive(&leftMotors, &rightMotors, 1.333, 3.25, 11.5, 12.5);
gen::setSensors(sensors, drive);
gen::setDistanceResetSensors({
  {&frontWall, gen::DistanceResetSensor::Side::Front, 0.0, 10.0},
  {&leftWall,  gen::DistanceResetSensor::Side::Left,  -4.0, 10.0},
});
gen::init();
auto pose = gen::getPose(); // degrees by default
```

<font size = 6>**Motion helpers**</font>
---
`gen::Motion` layers PID loops on top of odom for quick autonomous moves. It owns two `pros::MotorGroup` references (left/right) and two PID tunings (lateral + angular).

- tuning struct: `kP`, `kI`, `kD`, `headingToleranceDeg`, `settleTimeMs`, `maxCommand`, `minCommand`
- low level: `tank()`, `arcade()`, `stop()`, `getPose()`
- turn: `turnHeading(targetDeg, timeoutMs, minPower, maxPower)`, `turnPoint(x, y, ...)`
- drive: `movePoint(x, y, timeoutMs, minPower, maxPower, settle, forward)`
- convenience: `moveDistance(distanceInches, ...)`, `movePose(x, y, headingDeg, dLead, gLead, ...)`

example auton snippets:
```cpp
gen::Motion::Tuning lateral{8.0, 0.0, 0.0, 2.0, 250, 127.0, 5.0};
gen::Motion::Tuning angular{2.0, 0.0, 0.0, 2.0, 250, 127.0, 0.0};
gen::Motion motion(leftMotors, rightMotors, lateral, angular);

motion.moveDistance(24.0, 4000, true, 5.0);    // 24" forward
motion.turnHeading(90.0, 3000, 5.0);           // face 90 deg
motion.movePose(24.0, 36.0, 45.0, 2.0, 0.5);   // drive to pose with lead
```

<font size = 6>**PID + Math Functions**</font>
---
- `gen::PID` (in `src/gen/pid.cpp`): `update(error)` with optional integral windup guard (`iMax`) and sign-change reset; `reset()` clears integrator/derivative memory.
- `gen::kPi`, `deg_to_rad()`, `rad_to_deg()`, `ema(sample, prev, alpha)`, `getSign(auto)` live in `gen/misc`.

<font size = 6>**Exit conditions**</font>
-
---
Combine multiple stopping rules for motion loops.

- build: `ExitConditions exits(timeoutMs); exits.add("err", ...)`
- query: `shouldContinue()` returns false once any check fails or timeout hits
- factories: 
  - `errorAbove([]{ return error; }, minError)`
  - `velocityAbove([]{ return speed; }, minVelocity)`
  - `halfPlaneNotCrossed(poseSupplier, targetPose, headingDeg, tolerance)`

```cpp
gen::ExitConditions exits(3000);
exits.add("err", gen::ExitConditions::errorAbove([&]{ return target - measured; }, 0.25));
while (exits.shouldContinue()) {
  // drive loop...
}
```

<font size = 6>**Autonomous Selector**</font>
---
- `include/gen/auton.h` holds stub functions and a `routines` vector of `{name, fn}`. Fill these out or mirror the pattern in `src/main.cpp` where a limit switch steps through a list of routines.
- `gen::selector` headers are empty placeholders—drop your own UI or keep the simple LCD text approach used in `main.cpp`.

<font size = 6>**Notes**</font>
---
- `include/gen/colorsort.h` and `src/gen/colorsort.cpp` are placeholders; the prior color-sort logic is commented for reference.
- `src/gen/chassis.cpp` is commented out in favor of the richer `gen::Motion` layer; uncomment/extend if you prefer a lighter drivetrain wrapper.

### Any Questions or want to contribute? 
- Message 'nickson78181a' on discord!
