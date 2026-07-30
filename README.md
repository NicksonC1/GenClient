# <img src="images/genclient.png">

# GenClient 
(Will fill out later)

# GenSelector
GenSelector is a PROS + LVGL autonomous selector and match HUD for VEX V5.

This version is packaged as one folder:

- `src/GenSelector/selector.hpp`
- `src/GenSelector/selector.cpp`
- `src/GenSelector/background.c`
- `src/GenSelector/logosmall.c`
- `src/GenSelector/logo.c`

The intended setup is:

1. Copy the whole `src/GenSelector/` folder into your own PROS project's `src/`
2. Paste a few blocks into your `src/main.cpp`
3. Replace the example motors, auton functions, and telemetry getters with your own

This README explains exactly what to paste, where to paste it, and what each pasted block does.

---

## Setup First

Before copying any code, make sure your project has the same base tools this selector was built against.

### Required Versions

- PROS kernel: `4.2.2`
- liblvgl template: `9.2.0`
- Target: `V5`

This repo's `project.pros` is currently using:

- `kernel@4.2.2`
- `liblvgl@9.2.0`

If you use older or different versions, the selector may still work, but you are more likely to hit:

- missing LVGL symbols
- image type mismatches
- event/callback API mismatches
- runtime UI issues

### If You Do Not Have PROS Installed

Install PROS first, then create a normal V5 C++ project before adding GenSelector.

Recommended install path:

1. Install Visual Studio Code
2. Install the official PROS extension in VS Code
3. Let the extension install the PROS CLI/toolchain when prompted
4. Create a new V5 C++ PROS project

What you should end up with:

- a normal PROS V5 project
- `project.pros` in the project root
- `include/pros/`
- `include/liblvgl/`
- `src/main.cpp`

### If You Already Have PROS But Not LVGL

GenSelector requires the PROS `liblvgl` template.

Check your project first:

- if you already have `include/liblvgl/`, LVGL is already present
- if `include/liblvgl/` is missing, add/install the `liblvgl` template before using GenSelector

Practical rule:

- if you are unsure, create a fresh PROS 4 V5 project with LVGL included, then copy `src/GenSelector/` into that project

### Recommended Baseline

The safest setup is:

1. Create a fresh PROS V5 C++ project
2. Make sure it uses PROS 4
3. Make sure `liblvgl` is installed
4. Confirm these folders exist:

```txt
include/pros/
include/liblvgl/
src/
```

Only after that should you copy in GenSelector and paste the `main.cpp` setup.

---

## What It Does

GenSelector gives you:

- A brain-screen autonomous selector
- A custom LVGL UI with team number and Gen branding
- Three bottom-left temperature gauges
- Three telemetry lines for values like `X`, `Y`, and `Theta`
- A selected autonomous that runs in `autonomous()`

Current selector behavior:

- Tap the highlighted/current option to go to the next auton
- Tap the faded option above to go to the previous auton
- The battery percentage at the top uses the real brain battery level

---

## What To Copy

Copy this folder into your own project:

```txt
src/GenSelector/
```

After copying, your project should contain:

```txt
src/
  main.cpp
  GenSelector/
    selector.hpp
    selector.cpp
    background.c
    logosmall.c
    logo.c
```

You do not need to copy anything into `include/`.

---

## What To Paste In `main.cpp`

You need to paste five things into `src/main.cpp`:

1. The include
2. Your motors and telemetry getters
3. Your autonomous routine list
4. The selector config and selector object
5. The lifecycle hooks in `initialize()` and `autonomous()`

---

## 1. Paste The Include

Paste this near the top of `src/main.cpp`, with your other includes:

```cpp
#include "GenSelector/selector.hpp"
```

What it does:

- Includes the selector library from the copied `src/GenSelector/` folder

Where to paste it:

- At the top of `src/main.cpp`
- Usually under `#include "main.h"`

Example:

```cpp
#include "main.h"

#include <algorithm>

#include "GenSelector/selector.hpp"
#include "pros/screen.hpp"
```

---

## 2. Paste Your Motors And Telemetry Getters

Paste your drivetrain/subsystem motors and your telemetry getter functions near the top of `main.cpp`, before the auton list/config.

Example:

```cpp
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({18, -19, 20}, pros::MotorGearset::blue);

pros::Motor intake(-14, pros::MotorGearset::blue);
pros::Motor indexer(17, pros::MotorGearset::blue);

double poseX = 24.0;
double poseY = -24.0;
double poseTheta = 45.0;

double getX() { return poseX; }
double getY() { return poseY; }
double getTheta() { return poseTheta; }
```

What it does:

- `leftMotors`, `intake`, and `indexer` are used by the temperature gauges
- `getX`, `getY`, and `getTheta` are used by the left-side telemetry box

What you should replace:

- Replace the example ports with your actual motor ports
- Replace `poseX`, `poseY`, `poseTheta` with your real odometry/chassis getters

For example, if you use a chassis library:

```cpp
double getX() { return chassis.getPose().x; }
double getY() { return chassis.getPose().y; }
double getTheta() { return chassis.getPose().theta; }
```

Important:

- The telemetry fields use function pointers, so use functions like `getX()`, not direct values like `chassis.getPose().x`

---

## 3. Paste Your Autonomous Functions And List

Paste your auton functions and your auton vector before the selector config.

Example:

```cpp
using robot::AutonFunc;

namespace Auton {

void test() { }
void left() { }
void right() { }
void skills() { }
void test1() { }
void test2() { }
void test3() { }

}  // namespace Auton

robot::AutonRoutineList autonRoutines = {
    {"Default Auton", static_cast<AutonFunc>(Auton::test)},
    {"Left", static_cast<AutonFunc>(Auton::left)},
    {"Right", static_cast<AutonFunc>(Auton::right)},
    {"Skills", static_cast<AutonFunc>(Auton::skills)},
    {"Test1", static_cast<AutonFunc>(Auton::test1)},
    {"Test2", static_cast<AutonFunc>(Auton::test2)},
    {"Test3", static_cast<AutonFunc>(Auton::test3)},
};
```

What it does:

- Defines the routines shown on the right side of the selector
- Connects each menu label to a function that runs in `autonomous()`

What you should replace:

- Replace the example auton function bodies with your real autons
- Replace the labels with your actual names

Example:

```cpp
robot::AutonRoutineList autonRoutines = {
    {"Left Quals", static_cast<AutonFunc>(Auton::leftQuals)},
    {"Right Rush", static_cast<AutonFunc>(Auton::rightRush)},
    {"Solo AWP", static_cast<AutonFunc>(Auton::soloAwp)},
    {"Skills", static_cast<AutonFunc>(Auton::skills)},
};
```

---

## 4. Paste The Selector Config And Object

Paste this after your auton list.

Example:

```cpp
const robot::SelectorConfig autonSelectorConfig{
    .input = {
        .type = robot::SelectorInputType::BrainScreen,
    },
    .menu = {
        .teamNumber = "78181A",
    },
    .devices = robot::SelectorDevicesConfig(
        {"Chassis", &leftMotors},
        {"Intake", &intake},
        {"Indexer", &indexer}
    ),
    .terminal = {
        .fields = {
            {"X", getX, 2},
            {"Y", getY, 2},
            {"Theta", getTheta, 2},
        },
        .refreshMs = 50,
    },
    .lcdLine = 4,
    .pollDelayMs = 20,
};

robot::AutonSelector autonSelector(autonSelectorConfig, autonRoutines);
```

What it does:

- Chooses how the selector is controlled
- Sets the team number shown at the top
- Defines the three temperature gauges
- Defines the three telemetry lines
- Creates the selector object itself

What you should replace:

- `.teamNumber`
- `leftMotors`, `intake`, `indexer`
- `getX`, `getY`, `getTheta`

### What `.devices` means

This block:

```cpp
.devices = robot::SelectorDevicesConfig(
    {"Chassis", &leftMotors},
    {"Intake", &intake},
    {"Indexer", &indexer}
),
```

means:

- Bottom-left gauge 1 label = `Chassis`, value comes from `&leftMotors`
- Bottom-left gauge 2 label = `Intake`, value comes from `&intake`
- Bottom-left gauge 3 label = `Indexer`, value comes from `&indexer`

If a source is a `pros::MotorGroup`, the selector reads the configured motor index from that group.

### What `.terminal.fields` means

This block:

```cpp
.fields = {
    {"X", getX, 2},
    {"Y", getY, 2},
    {"Theta", getTheta, 2},
},
```

means:

- Show `X` with 2 decimal places
- Show `Y` with 2 decimal places
- Show `Theta` with 2 decimal places

The selector calls those getter functions repeatedly while the UI is running.

### Input modes

For brain screen touch, use:

```cpp
.input = {
    .type = robot::SelectorInputType::BrainScreen,
},
```

That is the default setup for the current project.

---

## 5. Paste The PROS Lifecycle Hooks

Paste the selector start in `initialize()`:

```cpp
void initialize() {
    autonSelector.start();
}
```

Paste the selected auton run in `autonomous()`:

```cpp
void autonomous() {
    autonSelector.runSelected(Auton::test);
}
```

What it does:

- `autonSelector.start()` builds the LVGL screen and starts the selector task
- `runSelected(...)` runs the chosen routine, or falls back if needed

What you should replace:

- Replace `Auton::test` with your preferred fallback auton

---

## Full Example `main.cpp`

This is the current example structure used in this repo:

```cpp
#include "main.h"

#include <algorithm>

#include "GenSelector/selector.hpp"
#include "pros/screen.hpp"

using robot::AutonFunc;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({18, -19, 20}, pros::MotorGearset::blue);

pros::Motor intake(-14, pros::MotorGearset::blue);
pros::Motor indexer(17, pros::MotorGearset::blue);

double poseX = 24.0;
double poseY = -24.0;
double poseTheta = 45.0;

double getX() { return poseX; }
double getY() { return poseY; }
double getTheta() { return poseTheta; }

namespace Auton {

void test() { }
void left() { }
void right() { }
void skills() { }
void test1() { }
void test2() { }
void test3() { }

}  // namespace Auton

robot::AutonRoutineList autonRoutines = {
    {"Default Auton", static_cast<AutonFunc>(Auton::test)},
    {"Left", static_cast<AutonFunc>(Auton::left)},
    {"Right", static_cast<AutonFunc>(Auton::right)},
    {"Skills", static_cast<AutonFunc>(Auton::skills)},
    {"Test1", static_cast<AutonFunc>(Auton::test1)},
    {"Test2", static_cast<AutonFunc>(Auton::test2)},
    {"Test3", static_cast<AutonFunc>(Auton::test3)},
};

const robot::SelectorConfig autonSelectorConfig{
    .input = {
        .type = robot::SelectorInputType::BrainScreen,
    },
    .menu = {
        .teamNumber = "78181A",
    },
    .devices = robot::SelectorDevicesConfig(
        {"Chassis", &leftMotors},
        {"Intake", &intake},
        {"Indexer", &indexer}
    ),
    .terminal = {
        .fields = {
            {"X", getX, 2},
            {"Y", getY, 2},
            {"Theta", getTheta, 2},
        },
        .refreshMs = 50,
    },
    .lcdLine = 4,
    .pollDelayMs = 20,
};

robot::AutonSelector autonSelector(autonSelectorConfig, autonRoutines);

void initialize() {
    autonSelector.start();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    autonSelector.runSelected(Auton::test);
}

void opcontrol() {
    while (true) {
        pros::delay(10);
    }
}
```

---

## Common Mistakes

### 1. Putting the folder in `include/`

Do not put the whole selector folder in `include/`.

Use:

```txt
src/GenSelector/
```

Reason:

- `selector.cpp` and the `.c` image files need to be compiled as source files

### 2. Using direct values in `.fields`

This is wrong:

```cpp
{"X", chassis.getPose().x, 2}
```

Use a getter function instead:

```cpp
double getX() { return chassis.getPose().x; }
```

and then:

```cpp
{"X", getX, 2}
```

### 3. Expecting LVGL `%f` formatting to work

This project already avoids that internally.

If you see `f` on screen, it means some other part of your project is still using float formatting through an embedded `printf` path that does not support it.

### 4. Wrong asset symbol names

The image files need to expose:

- `background`
- `logosmall`

If those names change, the selector will not link correctly.

---

## Files You Usually Edit

- [main.cpp](/Users/nicksonc/Desktop/VexProjects/GenSelector/src/main.cpp)
- [selector.cpp](/Users/nicksonc/Desktop/VexProjects/GenSelector/src/GenSelector/selector.cpp)
- [selector.hpp](/Users/nicksonc/Desktop/VexProjects/GenSelector/src/GenSelector/selector.hpp)

Most users only need to edit `main.cpp`.

---

## GenClient

GenSelector is intended to be used alongside GenClient.

Typical pattern:

- GenClient handles chassis, odom, motion, and robot systems
- GenSelector handles autonomous selection and brain-screen display

If you already have GenClient running, replace the example telemetry getters with your real chassis pose getters and replace the example auton vector with your real routines.

## Questions/Contributions?

Message ```nickson78181a``` on Discord!
