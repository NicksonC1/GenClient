#include "main.h"
#include "gen/electronics.h"
#include "gen/setup.hpp"
#include "pros/distance.hpp"

using DriveMode = gen::Controller::DriveMode;
using Button = gen::Controller::Button;

// ========================= User Template =========================
// Edit this section first when reusing the project on a new robot.
// 1) Update motor/sensor ports.
// 2) Update drivetrain geometry and wheel RPM.
// 3) Update PID constants after tuning.

gen::Controller controller(gen::Controller::DriveMode::Arcade2Stick, 3, 10.0, true);
gen::MotorGroup leftDrive({1, -2, 3}, 600.0, 1.0);
gen::MotorGroup rightDrive({-4, 5, -6}, 600.0, 1.0);
gen::CustomIMU imu(9, 1.01123595506);

gen::Piston wingPiston('A', false, "wing");
gen::PistonGroup wings({{"wing", &wingPiston}}); 

constexpr gen::Motion::DrivetrainProfile drivetrainProfile{
    .trackWidthIn = 11.75f,
    .wheelDiameterIn = gen::Omniwheel::NEW_325,
    .wheelRpm = 600.0f,
    .horizontalDrift = 10.0f,
};

constexpr gen::Motion::ControllerProfile lateralProfile{
    .gains = {7.5f, 0.0f, 6.0f},
    .antiWindupRange = 0.0f,
    .smallError = {1.0f, 100},
    .largeError = {3.0f, 500},
    .slew = 0.0f,
};

constexpr gen::Motion::ControllerProfile angularProfile{
    .gains = {2.75f, 0.0f, 17.5f},
    .antiWindupRange = 0.0f,
    .smallError = {1.0f, 100},
    .largeError = {3.0f, 500},
    .slew = 0.0f,
};

constexpr gen::Motion::OdomProfile odomProfile{
    .imu = &imu,
};
// ======================= End User Template =======================

gen::Drivetrain drivetrain = drivetrainProfile.toGen(&leftDrive, &rightDrive);
gen::ControllerSettings lateralController = lateralProfile.toGen();
gen::ControllerSettings angularController = angularProfile.toGen();
gen::OdomSensors odomSensor = odomProfile.toGen();
gen::Chassis chassis(drivetrain, lateralController, angularController, odomSensor);

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    // controller.raw().rumble(".");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        controller.arcade_two_stick();

        // Sample: toggle wings with R2 pressed.
        // Each new R2 press flips wings on/off.
        if(controller.pressed(Button::R2)) { wings.toggle(); }
        pros::delay(10);
    }
}
