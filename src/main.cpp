#include "main.h"
#include "gen/colorsort.h"
#include "gen/electronics.h"
#include "gen/odom.h"
#include "gen/motion.h"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "gen/exit.h"

using DriveMode = gen::Controller::DriveMode;
using Button = gen::Controller::Button;
 // <--------------------------------------------------------------- Setup ------------------------------------------------------------------>
gen::Controller controller(DriveMode::Arcade2Stick, 3, 10.0, true, pros::E_CONTROLLER_MASTER);
gen::Piston wingPiston('A', false, "wing");
gen::PistonGroup wings({{"wing", &wingPiston}}); 

pros::adi::DigitalIn autonSwitch('H');
pros::Distance frontWall(4);
pros::Distance leftWall(3);

pros::Rotation verticalLeft(5);
pros::Rotation verticalRight(6);
pros::Rotation horizontal(7);
gen::CustomIMU s_imu(9, 1.0);

gen::MotorGroup leftMotors({-11, -12, -13}, 600.0, 1.333); 
gen::MotorGroup rightMotors({18, 19, 20}, 600.0, 1.333); 

gen::TrackingWheel verticalLTracker(&verticalLeft, 2.75, 3.5, 1.0, false);
gen::TrackingWheel verticalRTracker(&verticalRight, 2.75, -3.5, 1.0, true);
gen::TrackingWheel horizontalTracker(&horizontal, 2.75, -2.5);

gen::OdomSensors sensors{&verticalLTracker, &verticalRTracker, &horizontalTracker, nullptr, &s_imu};
gen::Drivetrain drive(&leftMotors, &rightMotors, 1.333, 3.25, 11.5, 12.5);

std::vector<gen::DistanceResetSensor> distanceResetSensors = {
    {&frontWall, gen::DistanceResetSensor::Side::Front, 0.0, 10.0},
    {&leftWall, gen::DistanceResetSensor::Side::Left, -4.0, 10.0},
};

gen::Motion::Tuning lateralTuning{8.0,   // kP: proportional gain for linear error
                                  0.0,   // kI: integral gain for linear error
                                  0.0,   // kD: derivative gain for linear error
                                  2.0,   // headingToleranceDeg: acceptable angular error while driving
                                  250,   // settleTimeMs: how long error must stay within tolerance
                                  127.0  // maxCommand: clamp for motor command
};
gen::Motion::Tuning angularTuning{2.0,   // kP: proportional gain for turns
                                  0.0,   // kI: integral gain for turns
                                  0.0,   // kD: derivative gain for turns
                                  2.0,   // headingToleranceDeg: acceptable heading error when turning
                                  250,   // settleTimeMs: how long heading must stay within tolerance
                                  127.0  // maxCommand: clamp for motor command
};

gen::Motion motion(leftMotors, rightMotors, lateralTuning, angularTuning);

// gen::ExitConditions exits;

namespace Auton{
    void main(){
      // Drive forward 24" relative to current heading.
      {
        const auto pose = motion.getPose(true);
        const double targetX = pose.x + 24.0 * std::sin(pose.theta);
        const double targetY = pose.y + 24.0 * std::cos(pose.theta);
        motion.movePoint(targetX, targetY, 4000, 5.0, 127.0, true);
      }

      // Turn to absolute heading 90 deg.
      motion.turnHeading(90.0, 3000, 5.0, 127.0);

      // Strafe left 12" relative to new heading.
      {
        const auto pose = motion.getPose(true);
        const double targetX = pose.x + (-12.0) * -std::cos(pose.theta);
        const double targetY = pose.y + (-12.0) * std::sin(pose.theta);
        motion.movePoint(targetX, targetY, 4000, 5.0, 127.0, true);
      }

      // Move to a field pose with final heading 45 deg.
      motion.movePose(24.0, 36.0, 45.0, 2.0, 0.5, 5000, true, 5.0, 127.0, true);
    }
    void leftB(){}
    void leftR(){}
    void rightB(){}
    void rightR(){}
    void soloB(){}
    void soloR(){}
    void skills(){}
} // namespace Auton


int autonState = 0;

using AutonFunc = void(*)();
std::vector<std::pair<std::string, AutonFunc>> autonRoutines = {
  {"Default Auton", Auton::main},
  
  {"Blue Left", Auton::leftB},
  {"Red Left", Auton::leftR},

  {"Blue Right", Auton::rightB},
  {"Red Right", Auton::rightR},

  {"Blue Solo", Auton::soloB},
  {"Red Solo", Auton::soloR},

  {"Skills", Auton::skills},
};

void selector() {
  if (autonSwitch.get_new_press()) { autonState++; if (autonState == autonRoutines.size()) autonState = 0; }
  pros::lcd::set_text(4, autonRoutines[autonState].first);
  pros::delay(10);
}

// <----------------------------------------------------------- Main Functions ------------------------------------------------------------>
void initialize() {
  pros::Task t_Select(selector);
	pros::lcd::initialize();
  gen::setSensors(sensors, drive);
  gen::setDistanceResetSensors(distanceResetSensors);
  gen::init();

  // Example: access a single motor inside the group for diagnostics.
  leftMotors[1].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  pros::lcd::print(3, "Left mid temp: %.1f", leftMotors[1].get_temperature());

  pros::Task screenTask([&]() {
    while (1) {
        const auto pose = motion.getPose();
        pros::lcd::print(0, "X: %f", pose.x);
        pros::lcd::print(1, "Y: %f", pose.y);
        pros::lcd::print(2, "Theta: %f", pose.theta);
        pros::delay(50);
      }
  });

  pros::Task autonSelect([]{ while(1){ selector(); pros::delay(10); }});
}

void disabled() {}
void competition_initialize() {}
void autonomous() {
  (autonState < autonRoutines.size()) ? autonRoutines[autonState].second() : Auton::main();
}

void opcontrol() {
	while (1) {
    controller.arcade_two_stick();
    if(controller.pressed(Button::A)) { wings.toggle(); }
		pros::delay(10);
	}
}
