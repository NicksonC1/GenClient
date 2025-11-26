#include "main.h"
#include "gen/chassis.h"
#include "gen/colorsort.h"
#include "gen/electronics.h"
#include "gen/odom.h"
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

pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue); 

gen::TrackingWheel verticalLTracker(&verticalLeft, 2.75, 3.5, 1.0, false);
gen::TrackingWheel verticalRTracker(&verticalRight, 2.75, -3.5, 1.0, true);
gen::TrackingWheel horizontalTracker(&horizontal, 2.75, -2.5);

gen::OdomSensors sensors{&verticalLTracker, &verticalRTracker, &horizontalTracker, nullptr, &s_imu};
gen::Drivetrain drive(&leftMotors, &rightMotors, 1.333, 3.25, 11.5, 12.5);

std::vector<gen::DistanceResetSensor> distanceResetSensors = {
    {&frontWall, gen::DistanceResetSensor::Side::Front, 0.0, 10.0},
    {&leftWall, gen::DistanceResetSensor::Side::Left, -4.0, 10.0},
};

gen::Chassis::Tuning lateralTuning{8.0,   // kP: proportional gain for linear error
                                   0.0,   // kI: integral gain for linear error
                                   0.0,   // kD: derivative gain for linear error
                                   2.0,   // headingToleranceDeg: acceptable angular error while driving
                                   250,   // settleTimeMs: how long error must stay within tolerance
                                   4000,  // timeoutMs: max time to try the move
                                   127.0  // maxCommand: clamp for motor command
};
gen::Chassis::Tuning angularTuning{2.0,   // kP: proportional gain for turns
                                   0.0,   // kI: integral gain for turns
                                   0.0,   // kD: derivative gain for turns
                                   2.0,   // headingToleranceDeg: acceptable heading error when turning
                                   250,   // settleTimeMs: how long heading must stay within tolerance
                                   3000,  // timeoutMs: max time to try the turn
                                   127.0  // maxCommand: clamp for motor command
};

gen::Chassis chassis(leftMotors, rightMotors, drive, lateralTuning, angularTuning);

// gen::ExitConditions exits;

namespace Auton{
    void main(){
      chassis.driveDistance(24.0);
      chassis.turnToHeading(90.0);
      chassis.strafeDistance(-12.0);
      chassis.driveToPose({24.0, 36.0, 45.0});
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
  
  {"Blue Left Qual", Auton::leftB},
  {"Red Left Qual", Auton::leftR},

  {"Blue Right Qual", Auton::rightB},
  {"Red Right Qual", Auton::rightR},

  {"Blue Solo Qual", Auton::soloB},
  {"Red Solo Qual", Auton::soloR},

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

  pros::Task screenTask([&]() {
    while (1) {
        pros::lcd::print(0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
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
