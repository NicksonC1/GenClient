#include "main.h"
#include "gen/electronics.h"
#include "robot/motion_setup.hpp"

// ========================= User Template =========================
// Edit this section first when reusing the project on a new robot.
// 1) Update motor/sensor ports.
// 2) Update drivetrain geometry and wheel RPM.
// 3) Update PID constants after tuning.

gen::Controller controller(gen::Controller::DriveMode::Arcade2Stick, 3, 10.0, true);
gen::MotorGroup leftDrive({1, -2, 3}, 600.0, 1.0);
gen::MotorGroup rightDrive({-4, 5, -6}, 600.0, 1.0);
pros::Motor intake(10, pros::MotorGearset::blue);
pros::Imu imu(7);

constexpr robot::motion::DrivetrainProfile drivetrainProfile{
    .trackWidthIn = 11.75f,
    .wheelDiameterIn = gen::Omniwheel::NEW_325,
    .wheelRpm = 600.0f,
    .horizontalDrift = 10.0f,
};

constexpr robot::motion::ControllerProfile lateralProfile{
    .gains = {7.5f, 0.0f, 6.0f},
    .antiWindupRange = 0.0f,
    .smallError = {1.0f, 100},
    .largeError = {3.0f, 500},
    .slew = 0.0f,
};

constexpr robot::motion::ControllerProfile angularProfile{
    .gains = {2.75f, 0.0f, 17.5f},
    .antiWindupRange = 0.0f,
    .smallError = {1.0f, 100},
    .largeError = {3.0f, 500},
    .slew = 0.0f,
};

constexpr robot::motion::OdomProfile odomProfile{
    .imu = &imu,
};
// ======================= End User Template =======================

gen::Drivetrain drivetrain = drivetrainProfile.toGen(&leftDrive, &rightDrive);
gen::ControllerSettings lateralController = lateralProfile.toGen();
gen::ControllerSettings angularController = angularProfile.toGen();
gen::OdomSensors chassisSensors = odomProfile.toGen();
gen::Chassis chassis(drivetrain, lateralController, angularController, chassisSensors);
bool intakeOn = false;

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    controller.raw().rumble(".");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        const auto [leftPower, rightPower] = controller.drive_values();
        chassis.tank(leftPower, rightPower);

        // Sample: toggle intake with L1 pressed.
        // Each new L1 press flips intake on/off.
        if (controller.pressed(gen::Controller::Button::L1)) {
            intakeOn = !intakeOn;
        }
        intake.move(intakeOn ? 127 : 0);

        pros::delay(10);
    }
}
