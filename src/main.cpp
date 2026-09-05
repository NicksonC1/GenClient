#include "main.h"
#include "GenSelector/selector.hpp"
#include "gen/electronics.h"
#include "gen/setup.hpp"
#include "pros/distance.hpp"

#include <algorithm>
#include <cmath>

using DriveMode = gen::Controller::DriveMode;
using Button = gen::Controller::Button;

// ========================= User Template =========================
// Edit this section first when reusing the project on a new robot.
// 1) Update motor/sensor ports.
// 2) Update drivetrain geometry and wheel RPM.
// 3) Update PID constants after tuning.

gen::Controller controller(gen::Controller::DriveMode::Arcade2Stick, 3, 10.0, false);
gen::MotorGroup leftDrive({8, -9}, 600.0, 1.33);
gen::MotorGroup rightDrive({2, -3}, 600.0, 1.33);
gen::MotorGroup cascade({10, -1}, 200.0, 1.0);
gen::MotorGroup intake({4}, 600.0, 1.0);
gen::MotorGroup intakeClaw({-6}, 200.0, 0.25);
gen::MotorGroup clawLift({-5}, 200.0, 0.25);
pros::Rotation horizontalEncoder(-22);
pros::Rotation verticalEncoder(-22);
gen::CustomIMU imu(21, 1.0); // 1.01123595506

// Measure these signed offsets from the robot's tracking center.
// Left/back offsets are negative; right/front offsets are positive.
gen::TrackingWheel verticalTrackingWheel(&verticalEncoder, 2.0, -1.0);
gen::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, 2.75, -2.469176);

gen::Piston wingPiston('A', false, "wing");
gen::PistonGroup wings({{"wing", &wingPiston}}); 

void driveHold(gen::MotorGroup& motors, Button forward, Button reverse, double speed = 1.0) {
    if (controller.holding(forward)) {
        motors.move_percent(speed);
    } else if (controller.holding(reverse)) {
        motors.move_percent(-speed);
    } else {
        motors.move_percent(0);
    }
}

extern gen::Chassis chassis;

namespace OdomTune {
    bool active = false;
    int sampleCount = 0;
    int lastDirection = 0;
    double lastVertical = 0.0;
    double lastHorizontal = 0.0;
    double lastHeadingDeg = 0.0;
    double lastVerticalOffset = 0.0;
    double lastHorizontalOffset = 0.0;
    double avgVerticalOffset = 0.0;
    double avgHorizontalOffset = 0.0;

    constexpr double targetDegrees = 720.0;
    constexpr int minSpinPower = 18;
    constexpr int maxSpinPower = 42;
    constexpr int timeoutMs = 7000;
    constexpr int delayMs = 10;

    const char* status() {
        if (active) return "RUNNING";
        if (sampleCount > 0) return "READY";
        return "IDLE";
    }

    const char* directionLabel() {
        if (lastDirection > 0) return "CCW";
        if (lastDirection < 0) return "CW";
        return "--";
    }

    void run(int direction) {
        if (active || direction == 0) return;

        active = true;
        lastDirection = direction;

        leftDrive.move(0);
        rightDrive.move(0);
        pros::delay(100);

        verticalTrackingWheel.reset();
        horizontalTrackingWheel.reset();
        imu.tare_rotation();
        chassis.setPose(0, 0, 0);
        pros::delay(50);

        int elapsed = 0;
        while (elapsed < timeoutMs && std::abs(imu.get_rotation()) < targetDegrees) {
            const double remaining = targetDegrees - std::abs(imu.get_rotation());
            const int power = std::clamp(static_cast<int>(remaining * 0.08) + minSpinPower, minSpinPower, maxSpinPower);
            leftDrive.move(direction * power);
            rightDrive.move(-direction * power);
            pros::delay(delayMs);
            elapsed += delayMs;
        }

        leftDrive.brake();
        rightDrive.brake();
        pros::delay(300);

        lastVertical = verticalTrackingWheel.getDistanceTraveled();
        lastHorizontal = horizontalTrackingWheel.getDistanceTraveled();
        lastHeadingDeg = imu.get_rotation();

        const double headingRad = lastHeadingDeg * M_PI / 180.0;
        if (std::abs(headingRad) > 1e-5) {
            lastVerticalOffset = -lastVertical / headingRad;
            lastHorizontalOffset = -lastHorizontal / headingRad;
            avgVerticalOffset = (avgVerticalOffset * sampleCount + lastVerticalOffset) / (sampleCount + 1);
            avgHorizontalOffset = (avgHorizontalOffset * sampleCount + lastHorizontalOffset) / (sampleCount + 1);
            sampleCount++;
        }

        chassis.setPose(0, 0, 0);
        active = false;
        controller.raw().rumble(".");
    }
}  // namespace OdomTune

constexpr gen::Motion::DrivetrainProfile drivetrainProfile{
    .trackWidthIn = 11.5f,
    .wheelDiameterIn = gen::Omniwheel::NEW_275,
    .wheelRpm = 450.0f,
    .horizontalDrift = 10.0f,
};

constexpr gen::Motion::ControllerProfile lateralProfile{
    .gains = {
        .proportional = {.initial = 7.5f, .final = 7.5f, .scale = 12.0f, .power = 1.0f},
        .kI = 0.0f,
        .kD = 0.01f,
        .integralRange = 6.0f,
        .integralSignReset = true,
    },
    .exits = {
        .timeout = 2000,
        .velocityExit = 6.0f,
        .errorExit = 0.6f,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
    },
};

constexpr gen::Motion::ControllerProfile angularProfile{
    .gains = {
        .proportional = {.initial = 3.0f, .final = 3.0f, .scale = 28.0f, .power = 1.5f},
        .kI = 0.0f,
        .kD = 0.17f,
        .integralRange = 5.0f,
        .integralSignReset = true,
    },
    .correctionGains = {
        .proportional = {.initial = 0.0f, .final = 0.0f, .scale = 1.0f, .power = 1.0f},
        .kD = 0.0f,
    },
    .exits = {
        .timeout = 2000,
        .velocityExit = 3.0f,
        .errorExit = 4.0f,
        .halfPlaneExit = false,
    },
};

// constexpr gen::Motion::ControllerProfile lateralProfile{
//     .gains = {
//         .proportional = {.initial = 5.73f, .final = 5.73f, .scale = 12.0f, .power = 1.0f},
//         .kI = 0.88f,
//         .kD = 0.01f,
//         .integralRange = 6.0f,
//         .integralSignReset = true,
//     },
//     .exits = {
//         .timeout = 2000,
//         .velocityExit = 6.0f,
//         .errorExit = 0.6f,
//         .halfPlaneExit = true,
//         .halfPlaneTolerance = 2.4f,
//     },
// };

// constexpr gen::Motion::ControllerProfile angularProfile{
//     .gains = {
//         .proportional = {.initial = 4.75f, .final = 2.33f, .scale = 28.0f, .power = 1.5f},
//         .kI = 15.875f,
//         .kD = 0.157f,
//         .integralRange = 5.0f,
//         .integralSignReset = true,
//     },
//     .correctionGains = {
//         .proportional = {.initial = 0.0f, .final = 0.0f, .scale = 1.0f, .power = 1.0f},
//         .kD = 0.0f,
//     },
//     .exits = {
//         .timeout = 2000,
//         .velocityExit = 3.0f,
//         .errorExit = 4.0f,
//         .halfPlaneExit = false,
//     },
// };

constexpr gen::Motion::OdomProfile odomProfile{
    .vertical1 = nullptr,
    .vertical2 = nullptr,
    .horizontal1 = nullptr,
    .horizontal2 = nullptr,
    .imu = &imu,
};
// ======================= End User Template =======================

gen::Drivetrain drivetrain = drivetrainProfile.toGen(&leftDrive, &rightDrive);
gen::ControllerSettings lateralController = lateralProfile.toGen();
gen::ControllerSettings angularController = angularProfile.toGen();
gen::OdomSensors odomSensor = odomProfile.toGen();
gen::Chassis chassis(drivetrain, lateralController, angularController, odomSensor);

double selectorX() { return chassis.getPose().x; }
double selectorY() { return chassis.getPose().y; }
double selectorTheta() { return chassis.getPose().theta; }

namespace Auton {

void Left() {

    chassis.setPose(-60.9, 0, 270);
    intakeClaw.move(127);
    clawLift.move_absolute(710, 80);
    // pros::delay(1500);
    

    
    // intakeClaw.move(127);
    // clawLift.move_absolute(510, 100);
    // cascade.move_absolute(90, 127);

    chassis.moveToPose(-69, 0, 270, {
        .timeout = 1100,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = true,
        .maxSpeed = 100,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = false,
    });
    chassis.tank(-40,-40);
    pros::delay(350);
    chassis.tank(0,0);
    pros::delay(150);
    chassis.moveToPose(-70, 0, 270, {
        .timeout = 1000,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = true,
        .maxSpeed = 110,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = false,
    });
    chassis.moveToPose(-53, 0, 270, {
        .timeout = 2000,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = false,
        .maxSpeed = 100,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = true,
    });

    chassis.turnToHeading(0, {.timeout = 1500, .velocityExit = -1, .errorExit = 2.0});
    chassis.moveToPose(-53, -22, 0, {
        .timeout = 2000,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = false,
        .maxSpeed = 100,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = false,
    });
    pros::delay(500);
    intakeClaw.move(-100);



    // chassis.moveToPoint(0, 24, {.timeout = 2000, .velocityExit = -1, .errorExit = -1, .forwards = true, .maxSpeed = 100, .halfPlaneExit = false, .halfPlaneTolerance = 2.4f});
    // chassis.moveToPose(24, 24, 90, {.timeout = 2000, .velocityExit = -1, .errorExit = -1, .halfPlaneExit = false, .halfPlaneTolerance = 2});
    // chassis.turnToHeading(10,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1}); 
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1}); 
    // chassis.turnToHeading(20,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(30,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(45,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(60,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(75,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(90,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1, .maxSpeed = 127});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1, .maxSpeed = 127});
    // chassis.turnToHeading(120, {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(150, {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(180, {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.moveToPoint(0, 24);
    // chassis.moveToPoint(0, 0);
    // chassis.moveToPoint(24, 24);
    // chassis.moveToPoint(0, 0);

    // chassis.moveToPoint(24, 24, {.timeout = 2000, .velocityExit = 6, .errorExit = 0.0, .forwards = true, .maxSpeed = 100, .halfPlaneExit = true, .halfPlaneTolerance = 2.4f});
    // chassis.moveToPoint(0, 24, {.timeout = 2000, .velocityExit = 6, .errorExit = 0.0, .forwards = true, .maxSpeed = 100, .halfPlaneExit = true, .halfPlaneTolerance = 2.4f});
    // chassis.moveToPoint(24, 24, {.timeout = 2000, .velocityExit = 6, .errorExit = 0.0, .forwards = true, .maxSpeed = 100, .halfPlaneExit = true, .halfPlaneTolerance = 2.4f});
    // chassis.moveToPoint(0, 0, {.timeout = 2000, .velocityExit = 0, .errorExit = 0.0, .forwards = false, .maxSpeed = 100, .halfPlaneExit = true, .halfPlaneTolerance = 2.4f});

    // chassis.moveToPoint(24, 24, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPoint(0, 24, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPoint(24, 24, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPoint(0, 0, {
    //     .timeout = 2500,
    //     .velocityExit = 6.0f,
    //     .errorExit = 0.6f,
    //     .forwards = false,
    //     .maxSpeed = 100,
    //     .minSpeed = 0,
    //     .halfPlaneExit = false,
    //     .settle = true,
    // });




    // chassis.moveToPose(24, 24, 45, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPose(0, 24, 270, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPose(24, 24, 90, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // // Drive backward southwest while the robot's front faces northeast.
    // chassis.moveToPose(0, 0, 0, {
    //     .timeout = 2500,
    //     .velocityExit = 6.0f,
    //     .errorExit = 0.6f,
    //     .forwards = false,
    //     .maxSpeed = 100,
    //     .minSpeed = 0,
    //     .halfPlaneExit = false,
    //     .settle = true,
    // });


    // Assumes the robot starts at (0, 0, 0).

    // Assumes the robot starts at (0, 0, 0).

    // chassis.moveToPose(24, 24, 45, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .dLead = 8.0f,
    //     .gLead = 0.35f,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPose(0, 24, 245, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .dLead = 8.0f,
    //     .gLead = 0.35f,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPose(0, 0, 180, {
    //     .timeout = 2500,
    //     .velocityExit = 6.0f,
    //     .errorExit = 0.6f,
    //     .forwards = true,
    //     .dLead = 3.0f,
    //     .gLead = 0.0f,
    //     .maxSpeed = 100,
    //     .minSpeed = 0,
    //     .halfPlaneExit = false,
    //     .settle = true,
    // });

    //  chassis.turnToHeading(0, {.timeout = 1500, .velocityExit = 3, .errorExit = 4, .lockedSide = gen::LockedSide::RIGHT});
                                     
}
void Right() {

    chassis.setPose(0, -60.9, 180);
    intakeClaw.move(127);
    clawLift.move_absolute(710, 80);
    // pros::delay(1500);
    

    
    // intakeClaw.move(127);
    // clawLift.move_absolute(510, 100);
    // cascade.move_absolute(90, 127);

    chassis.moveToPose(0, -69, 180, {
        .timeout = 1100,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = true,
        .maxSpeed = 100,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = false,
    });
    chassis.tank(-40,-40);
    pros::delay(350);
    chassis.tank(0,0);
    pros::delay(150);
    chassis.moveToPose(0, -70, 180, {
        .timeout = 1000,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = true,
        .maxSpeed = 110,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = false,
    });
    chassis.moveToPose(0, -52, 180, {
        .timeout = 2000,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = false,
        .maxSpeed = 100,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = true,
    });
    pros::delay(100);

    chassis.turnToHeading(90, {.timeout = 1500, .velocityExit = -1, .errorExit = 2.0});
    chassis.moveToPose(-22, -51, 90, {
        .timeout = 2000,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = false,
        .maxSpeed = 100,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = false,
    });
    pros::delay(700);
    intakeClaw.move(-100);
}

}  // namespace Auton

robot::AutonRoutineList autonRoutines = {
    {"Left", static_cast<robot::AutonFunc>(Auton::Left)},
    {"Right", static_cast<robot::AutonFunc>(Auton::Right)},
};

const robot::SelectorConfig autonSelectorConfig{
    .input = {
        .type = robot::SelectorInputType::BrainScreen,
    },
    .menu = {
        .teamNumber = "78181A",
    },
    .devices = robot::SelectorDevicesConfig(
        {"Left Drive", &leftDrive, 0},
        {"Right Drive", &rightDrive, 0},
        {"Intake", &intake, 0}
    ),
    .terminal = {
        .fields = {
            // {"X", selectorX, 2},
            // {"Y", selectorY, 2},
            // {"Theta", selectorTheta, 2},

            {"X", selectorX, 2},
            {"Y", selectorY, 2},
            {"Theta", selectorTheta, 2},
        },
        .refreshMs = 50,
    },
    .lcdLine = 4,
    .pollDelayMs = 20,
};

robot::AutonSelector autonSelector(autonSelectorConfig, autonRoutines);

void initialize() {
    chassis.calibrate();
    chassis.setPose(-48, 0, 270);
    autonSelector.start();
    controller.raw().rumble(".");
    clawLift.tare_position();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    autonSelector.runSelected(Auton::Left);
    // Auton::Right();
}

void opcontrol() {
    // autonSelector.stop();
    // // if (!pros::lcd::is_initialized()) {
    //     pros::lcd::initialize();
    // // }

    // pros::Task screenTask([&]() {
    //     while (true) {
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x);
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y);
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
    //         pros::lcd::print(3, "Cascade %.2f Claw %.2f", cascade.get_position(), clawLift.get_position());
    //         pros::lcd::print(4, "Tune %s Dir:%s", OdomTune::status(), OdomTune::directionLabel());
    //         pros::lcd::print(5, "IMU %.1f V %.2f H %.2f", OdomTune::lastHeadingDeg, OdomTune::lastVertical, OdomTune::lastHorizontal);
    //         pros::lcd::print(6, "LastOff V %.2f H %.2f", OdomTune::lastVerticalOffset, OdomTune::lastHorizontalOffset);
    //         pros::lcd::print(7, "Avg%02d V %.2f H %.2f", OdomTune::sampleCount, OdomTune::avgVerticalOffset, OdomTune::avgHorizontalOffset);
    //         pros::delay(50);
    //     }
    // });

    intake.set_brake_mode_all(pros::MotorBrake::coast);
    cascade.set_brake_mode_all(pros::MotorBrake::hold);
    intakeClaw.set_brake_mode_all(pros::MotorBrake::coast);
    clawLift.set_brake_mode_all(pros::MotorBrake::hold);

    while (true) {
        const auto [leftOutput, rightOutput] = controller.arcade_two_stick();
        chassis.tank(leftOutput, rightOutput);

        // if (controller.pressed(Button::Up)) {
        //     OdomTune::run(1);
        // } else if (controller.pressed(Button::Left)) {
        //     OdomTune::run(-1);
        // }

        if (controller.holding(Button::L1)) {
            intake.move(127);
        } else if (controller.holding(Button::R1)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }

        if (controller.holding(Button::L2)) {
            cascade.move(127);
        } else if (controller.holding(Button::R2)) {
            cascade.move(-127);
        } else {
            cascade.move(0);
        }

        if (controller.holding(Button::Right)) {
            clawLift.move(127);
        } else if (controller.holding(Button::Y)) {
            clawLift.move(-127);
        } else {
            clawLift.set_brake_mode(pros::MotorBrake::hold);
            clawLift.brake();
            // clawLift.move(0);
        }

        if (controller.holding(Button::L1)) {
            intakeClaw.move(127);
        } else if (controller.holding(Button::A)) {
            intakeClaw.move(-127);
        } else {
            // intakeClaw.set_brake_mode(pros::MotorBrake::hold);
            // intakeClaw.brake();
            intakeClaw.move(0);
        }

        pros::delay(10);
    }
}
