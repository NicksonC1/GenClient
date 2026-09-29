# GenClient

<img src="images/genclient.png" alt="GenClient" width="320">

GenClient is a VEX V5 autonomous template built on PROS. It includes odometry, chassis movements, PID controllers with asymptotic proportional gains, and GenSelector for choosing an auton from the brain screen.

Most of your work happens in `src/main.cpp`: tell the template what hardware you have, tune it on your robot, then put movements together into a routine. The ports, gains, and autons already in that file belong to an example robot. You'll need to change them.

If you're setting this up for the first time, go in this order. Getting odometry right before tuning saves a lot of time.

1. [Open the project](#1-open-the-project)
2. [Configure your robot](#2-configure-your-robot)
3. [Check odometry](#3-check-odometry)
4. [Tune the controllers and Desmos curve](#4-tune-the-controllers)
5. [Learn the movements](#5-movements)
6. [Write and select an autonomous](#6-write-your-first-autonomous)
7. [Add chaining and mechanism timing](#7-chaining-and-running-mechanisms)
8. [Troubleshoot](#8-troubleshooting)
9. [GenSelector setup and reference](#9-genselector-setup-and-reference)

## 1. Open the project

Install VS Code and the PROS extension, then let the extension install its CLI and toolchain. The [PROS installation guide](https://pros.cs.purdue.edu/v5/getting-started/) covers that process.

Extract the template and open the **GenClient folder containing `project.pros`** in VS Code. This is already a complete project; you don't need to create another project or install LemLib into it.

The supplied project declares these dependencies:

| Dependency | Version in this template |
| --- | --- |
| Target | V5 |
| PROS kernel | 4.2.2 |
| liblvgl | 9.2.0 |

Keep the supplied `include/`, `firmware/`, `Makefile`, and `common.mk` together. GenSelector uses LVGL 9 APIs, so substituting an older LVGL installation can cause build errors.

Start with these files:

| File | What you'll use it for |
| --- | --- |
| `src/main.cpp` | Hardware, drivetrain dimensions, gains, autonomous routines, selector, driver control |
| `include/gen/chassis/chassis.hpp` | Every movement's parameters and defaults |
| `include/gen/pid.hpp` | PID configuration fields |
| `src/gen/chassis/motion.cpp` | Movement implementations, if you want to see exactly what a setting does |
| `src/GenSelector/` | The included selector and its image assets |
| `pid-simulator.html` | A browser-based PID simulator for experimenting with the gain curve |

After configuring the hardware below, use the PROS extension's build and upload actions. The extension also provides a combined build/upload button; see the [official extension page](https://marketplace.visualstudio.com/items?itemName=sigbots.pros). If you've already built from the terminal, `pros upload` uploads the project to the brain ([PROS upload guide](https://pros.cs.purdue.edu/v5/pros-4/uploading.html)).

## 2. Configure your robot

Open the `User Template` section near the top of `src/main.cpp`. Edit the existing declarations rather than adding a second set below them.

### Motors and mechanisms

A motor group looks like this:

```cpp
gen::MotorGroup leftDrive({8, -9}, 600.0, 600.0 / 450.0);
gen::MotorGroup rightDrive({2, -3}, 600.0, 600.0 / 450.0);
```

The arguments are:

1. **Ports.** A negative port reverses that motor. Left and right are from the robot's point of view, facing forward.
2. **Motor cartridge RPM.** Use `100`, `200`, or `600` for the cartridge actually installed.
3. **External gear ratio.** This wrapper uses motor RPM divided by wheel/output RPM. A 600 RPM motor driving a 450 RPM wheel uses `600.0 / 450.0`, or about `1.3333`.

Replace the example ports with your wiring. With a small positive command on both sides, every drive wheel should help move the robot forward. Fix reversed motors before running a motion.

Update the mechanism groups too: `cascade`, `intake`, `intakeClaw`, and `clawLift`. If your robot doesn't have one, remove its declaration **and its uses** in `initialize()`, `opcontrol()`, the selector, and the example autons.

For the motor wrapper, `move(127)` is full forward on the PROS output scale. `move_percent(1.0)` is also full forward; that helper uses `-1.0` to `1.0`, not `-100` to `100`.

Set piston ADI ports and initial states to match your robot as well. The supplied `wingPiston` uses ADI port `A`.

### Drivetrain dimensions

```cpp
constexpr gen::Motion::DrivetrainProfile drivetrainProfile{
    .trackWidthIn = 11.5f,
    .wheelDiameterIn = gen::Omniwheel::NEW_275,
    .wheelRpm = 450.0f,
    .horizontalDrift = 10.0f,
};
```

| Field | What to enter |
| --- | --- |
| `trackWidthIn` | Distance between the centers of the left and right drive wheels, in inches |
| `wheelDiameterIn` | Drive-wheel diameter in inches, or the matching `gen::Omniwheel` constant |
| `wheelRpm` | Wheel RPM **after external gearing**, not just the motor cartridge RPM |
| `horizontalDrift` | Stored by the drivetrain, but not used by the current motion implementation; leave it alone for now |

The motor wrapper's gear ratio and `wheelRpm` are separate settings. Changing one does not update the other. For the example above, both should describe a 600 RPM motor driving a 450 RPM wheel.

### IMU and tracking wheels

```cpp
gen::CustomIMU imu(21, 1.0);
```

The first argument is the IMU port. The second multiplies its rotation reading; start at `1.0`.

**The two `pros::Rotation` declarations in the supplied file both use `-22`. Those are placeholders, not usable V5 smart ports.** Use separate real ports from 1–21 if you have those sensors. The default odometry profile does not connect either tracking wheel, so declaring them alone doesn't enable them.

There are two useful starting configurations.

**Drive encoders + IMU:** keep the tracking-wheel pointers null, as they are in the supplied file.

```cpp
constexpr gen::Motion::OdomProfile odomProfile{
    .vertical1 = nullptr,
    .vertical2 = nullptr,
    .horizontal1 = nullptr,
    .horizontal2 = nullptr,
    .imu = &imu,
};
```

During calibration, GenClient fills missing vertical sensors with drive-motor encoder tracking wheels. This works without separate tracking wheels, but drive-wheel slip also becomes position error. Without a horizontal tracking wheel, the robot cannot measure sideways sliding.

**One vertical wheel + one horizontal wheel + IMU:** change the existing sensor ports and dimensions, then connect the wheels in the profile.

```cpp
// Example ports and measurements only. Replace your existing declarations.
pros::Rotation verticalEncoder(11);
pros::Rotation horizontalEncoder(12);

gen::TrackingWheel verticalTrackingWheel(&verticalEncoder, 2.0f, -1.0f);
gen::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, 2.75f, -2.5f);

constexpr gen::Motion::OdomProfile odomProfile{
    .vertical1 = &verticalTrackingWheel,
    .vertical2 = nullptr,
    .horizontal1 = &horizontalTrackingWheel,
    .horizontal2 = nullptr,
    .imu = &imu,
};
```

A vertical wheel rolls forward/backward. A horizontal wheel rolls sideways. The `TrackingWheel` arguments are the sensor pointer, wheel diameter in inches, and signed offset from the tracking center. For a vertical wheel, left is negative and right is positive. For a horizontal wheel, behind the center is negative and in front is positive. These are sideways or forward/backward offsets, not the diagonal distance to the wheel.

The optional fourth argument for a rotation-sensor tracking wheel is sensor revolutions per wheel revolution; leave it at `1.0` for direct drive.

If you use two external vertical wheels or two horizontal wheels, this implementation can use their differences for heading ahead of the IMU. Their offsets must describe their actual, distinct positions. The one-vertical/one-horizontal setup above uses the IMU for heading.

### Keep the startup code

`initialize()` already calls `chassis.calibrate()` before starting the selector. Keep the robot still while that runs. Calibration also starts the odometry task; motions need that task running.

Set your actual starting pose inside each auton. The pose in `initialize()` is just an initial display/reference pose, not a replacement for setting up each routine.

## 3. Check odometry

Odometry is the robot's estimate of where it is. Watch `X`, `Y`, and `Theta` on the included selector while checking it.

The coordinate system is:

| Value | Meaning |
| --- | --- |
| `x` | Inches to the right on your field map |
| `y` | Inches up your field map |
| `theta = 0` | Facing +Y |
| `theta = 90` | Facing +X |
| `theta = 180` | Facing −Y |
| `theta = 270` | Facing −X |

Headings increase clockwise. Negative headings can represent the same direction; `-90` and `270` point the same way.

```cpp
chassis.setPose(0, 0, 0);
```

This tells the robot where it is. It does not drive it there or physically square it to the field. Put the robot in the matching position before the test.

Before touching gains:

1. Start at `(0, 0, 0)` and move the robot forward a measured distance. `Y` should increase by about that distance.
2. Turn clockwise roughly 90°. The heading should increase by about 90°.
3. If you have a horizontal tracking wheel, move the robot to its right while facing 0°. `X` should increase. In this implementation, the horizontal wheel's raw distance increases toward the robot's **left**; reverse its sensor if the displayed X moves the wrong way.
4. Rotate around the tracking center without translating. X and Y should stay close to their starting values.
5. Repeat the distance and rotation checks in both directions.

A consistent distance scale error usually points to wheel diameter, gearing, or `wheelRpm`. A position jump during turns usually points to tracking-wheel direction or offset. Fix those first; a controller can't correct bad measurements.

For a wheel-diameter correction, use:

```text
new diameter = old diameter × actual distance / reported distance
```

Use a long, measured straight move and repeat it. Don't change gearing and diameter at the same time.

If the IMU consistently misreports a carefully measured full rotation, the corresponding scalar adjustment is:

```text
new scalar = old scalar × actual rotation / reported rotation
```

Use accumulated rotation, not a heading that wraps at 360°. Only do this after repeatable measurements.

There is an `OdomTune` helper in `main.cpp` that spins the robot and estimates tracking-wheel offsets using `-wheel travel / rotation in radians`. Its controller bindings and diagnostic display are commented out. It needs valid external tracking sensors first, and its displayed results must be copied into the wheel declarations manually. It does not save calibration for you. If you enable its old LCD display, stop the selector first so the two interfaces don't compete for the screen.

## 4. Tune the controllers

Don't start by tuning an entire auton. Make a small test routine with one movement, reset the robot to the same physical start, and change one setting at a time. Use the same surface and roughly the same load you'll run with.

GenClient has three controller roles:

| Settings in `main.cpp` | Used for |
| --- | --- |
| `lateralProfile.gains` | Forward/backward distance output |
| `angularProfile.gains` | Turns and steering during point/pose movements |
| `angularProfile.correctionGains` | Holding a heading during `moveDistance` and `crossBarrier` |

The supplied correction gains are all zero, so those straight-driving motions currently have **no active heading correction**. Turning gains don't automatically enable it.

### What P, I, and D do here

- **P** produces output proportional to error. More P usually makes the robot react harder; too much can make it overshoot or oscillate.
- **D** reacts to how quickly error is changing. It helps damp the approach; too much can make the response sluggish or noisy.
- **I** accumulates error near the target. It can help with a small persistent shortfall, but can also create overshoot. Leave it at zero while tuning P and D.

The controller runs on a nominal 10 ms loop. Its calculation is:

```text
integral += error × 0.01
 derivative = (error − previous error) × 100
 output = kP × error + kI × integral + kD × derivative
```

The integral resets outside `integralRange`, and can reset when error changes sign. Motor commands are then limited by the movement's speed settings and the `-127..127` output range.

That time scaling matters. Gains from a PID that simply adds error or subtracts consecutive errors won't transfer directly. Use this template's own tests rather than copying another library's numbers.

### First, get one fixed gain working

The proportional configuration is a curve, but you can make it constant by setting `initial` and `final` equal:

```cpp
.proportional = {
    .initial = 3.0f,
    .final = 3.0f,
    .scale = 28.0f,
    .power = 1.5f,
},
.kI = 0.0f,
.kD = 0.17f,
.integralRange = 5.0f,
.integralSignReset = true,
```

Those are the supplied angular values, not a promise that they'll work on your robot. While the endpoints are equal, `scale` and `power` don't affect P.

For an angular test, add this inside `namespace Auton` and add it to `autonRoutines` using the selector section below:

```cpp
void TuneTurn() {
    chassis.setPose(0, 0, 0); // physically start facing your 0° direction
    chassis.turnToHeading(90, {
        .timeout = 3000,
        .velocityExit = -1,
        .errorExit = -1,
        .maxSpeed = 80,
        .minSpeed = 0,
    });
}
```

Disabling both exits keeps the controller active until the finite timeout, so an early exit doesn't hide oscillation. It isn't the setup you'll use for a finished auton.

1. Set `kI = 0`. Start with a modest fixed P and little or no D.
2. If it barely moves or takes too long to approach, raise both `initial` and `final` together.
3. When it approaches quickly but overshoots, increase D in small steps.
4. If it repeatedly swings across the target, reduce P or add damping. If adding D only makes it slow and rough, back P down.
5. Once it gets there cleanly, repeat clockwise and counterclockwise.

Keep `maxSpeed` the same between comparisons. After tuning at reduced output, repeat at your intended auton speed. If nearly the whole move is clamped at the speed limit, raising P won't teach you much until the robot gets closer to the target.

For lateral tuning, use the same approach with `lateralProfile.gains` and a straight distance:

```cpp
void TuneDrive() {
    chassis.setPose(0, 0, 0);
    chassis.moveDistance(24, {
        .timeout = 3000,
        .velocityExit = -1,
        .errorExit = -1,
        .maxSpeed = 80,
        .minSpeed = 0,
    });
}
```

Check 6", 12", 24", and 48" moves, plus reverse moves. If it veers instead of driving straight, check the motors and heading correction before judging the distance tune.

### Collect points for Desmos

A fixed P may work well for a 90° turn and feel weak on a 10° turn. The asymptotic curve lets you choose different proportional gains for different movement sizes.

Once you have a usable D, hold D and the speed cap constant. Test several sizes with a fixed P each time. For turns, try 10°, 20°, 30°, 45°, 60°, 90°, 120°, and 180°. For each size, find a P that approaches well without repeated oscillation. Record repeated runs, not just the best one.

| Starting turn size | Fixed P that worked | Overshoot / settling notes |
| --- | --- | --- |
| 10° | Your result | |
| 30° | Your result | |
| 60° | Your result | |
| 90° | Your result | |
| 180° | Your result | |

These become points `(turn size, kP)` on the graph. The X axis is **movement size**, not time. The Y axis is **proportional gain**, not motor output, heading, or position.

For a lateral curve, make a separate table with distance in inches on the X axis. Don't mix inch and degree measurements in one fit.

### Set up the Desmos curve

Open the Desmos graphing calculator and enter the expression from the reference graph:

```text
y = (f - i) * (|x|^p / (|x|^p + k^p)) + i
```

Create sliders for `i`, `f`, `k`, and `p`. Keep `k > 0` and `p > 0`. Add a table containing your measured `(movement size, kP)` pairs, then adjust the curve to follow the useful trend.

| Desmos slider | Code field | What it controls |
| --- | --- | --- |
| `i` | `initial` | Gain at X = 0; the small-movement end of the curve |
| `f` | `final` | Gain the curve approaches for very large movements |
| `k` | `scale` | Movement size where the gain is halfway between `i` and `f` |
| `p` | `power` | How sharply the curve transitions around `k` |

`i` is just a graph variable here. It is **not** the integral gain `kI`. Likewise, graph `k` isn't a fifth PID gain; it becomes `scale`.

A few useful checks:

- At `x = 0`, `y = i`.
- At `x = k`, `y = (i + f) / 2`.
- As `|x|` gets large, `y` approaches `f`.
- Equal-size positive and negative movements get the same gain because the formula uses `|x|`.
- If `i = f`, the curve is flat.

Add `y = (i + f) / 2` as a horizontal guide, like the reference graph. Its intersection with the positive side of the curve tells you where `k` is.

Start by fitting the ends: `i` for the small-turn region and `f` for the large-turn region. Then move `k` to place the transition and adjust `p` to shape it. A larger `p` keeps the curve closer to its endpoints away from `k` and makes the transition around `k` sharper. The value of `f` is an asymptote; your 180° result doesn't have to equal it exactly.

Don't chase a perfect fit through every point. If one measurement is far off the trend, repeat that movement before bending the whole curve around it.

### Copy the curve into the profile

For an illustrative graph with `i = 4.5`, `f = 2.2`, `k = 28`, and `p = 1.5`, the matching code is:

```cpp
.proportional = {
    .initial = 4.5f,
    .final = 2.2f,
    .scale = 28.0f,
    .power = 1.5f,
},
```

This is an example of the mapping, not a finished tune. Replace the `.proportional` block under `angularProfile.gains` for turns, or under `lateralProfile.gains` for distance moves. Keep your tuned D alongside it.

**Use the same gain units in the graph and the code.** The reference screenshot shows values like `450` and `220`. This implementation does not divide them by 100. If those numbers represent gains multiplied by 100 for plotting, they must become `4.5` and `2.2` in code. If they aren't scaled measurements, don't assume that conversion. The easiest workflow is to plot the actual gains you tested, such as `4.5`, so there is nothing to convert.

### When the curve is sampled

This is worth understanding before spending hours fitting it. The PID selects a proportional gain when the motion calls `setTarget()`. It then keeps that gain while the error changes.

| Motion | X value used to choose the gain |
| --- | --- |
| Normal heading/point turn | Initial angular error, in degrees |
| Swing turn | Initial angular error divided by `2.5` |
| `moveDistance` | Requested distance, in inches |
| `moveToPoint` / `moveToPose` lateral controller | Fixed at `12` inches |
| `moveToPoint` / `moveToPose` angular controller | Fixed at `180` degrees |
| Straight-motion heading correction | Initial heading error |

For example, a normal 90° turn uses `curve(90)` throughout the turn. It does **not** slide along the curve toward `curve(0)` as the remaining error shrinks.

Point and pose movements therefore need their own checks after fitting the curve. Their steering uses the angular curve at 180°, even for a small correction. Swing turns also need separate tests because of their divided input. A good turn tune is a starting point, not proof that every movement is tuned.

### Heading correction

Tune `angularProfile.correctionGains` after the main turn and distance controllers work. Start with equal `initial` and `final`, `kI = 0`, and a modest P. Run a straight `moveDistance`, then add a little D if the heading hunts left and right.

The held heading comes from `setPose()` and previous motions. If you want a straight move along a new direction, turn there first. Since these motions often start with almost no heading error, their correction curve often selects a value close to `initial`; a fixed correction gain is a sensible first setup.

### Add integral only if you need it

If the robot repeatedly stops a little short while the controller is still active, first check friction, P/D, speed limits, and whether an exit is ending the move. If there's still a small persistent error:

1. Set `integralRange` slightly larger than the error you need to correct. Its unit is degrees for angular/correction gains and inches for lateral gains.
2. Keep `integralSignReset = true` so accumulated integral resets when error crosses zero.
3. Increase `kI` from zero in small steps.
4. Retest short and long moves. Back it down if the robot creeps through the target and then reverses.

Integral is cleared when error is outside `integralRange`. A range of zero effectively prevents useful accumulation for nonzero error. There is no separate integral-output cap here, so a large I can still accumulate during a long stall inside that range.

### Restore useful exits

Once the response is good, stop using the full timeout for every successful move:

```cpp
chassis.turnToHeading(90, {
    .timeout = 2000,
    .velocityExit = 3.0f,
    .errorExit = 2.0f,
});
```

With both enabled, this turn waits until the heading error is at most 2° **and** angular speed is at most 3°/s, or until the timeout ends it. These are immediate threshold checks; there is no required dwell time inside the range.

Make the error tolerance tight enough for the job without asking for precision the robot cannot repeat. A timeout is a fallback, not evidence that the target was reached.

Finally, rerun the full set of distances and angles, in both directions, with the load and speeds you intend to use. You can use `pid-simulator.html` to explore the response and copy a profile block, but its simulated robot won't capture your actual friction, backlash, wheel slip, or battery behavior. The robot gets the final say.

## 5. Movements

### How to fill out parameters

Required targets go first. Optional settings go inside `{}`:

```cpp
chassis.moveToPoint(12, 36, {
    .timeout = 2500,
    .forwards = true,
    .maxSpeed = 90,
    .halfPlaneExit = false,
});
```

Here, `(12, 36)` is an absolute field coordinate. Omitted settings use the parameter struct's defaults; omitted timeout/error/velocity settings inherit the appropriate controller profile.

Keep named fields in the order they're declared in `chassis.hpp`. C++ designated initializers are not an unordered dictionary. For example, put `.timeout` before `.maxSpeed`, and `.async` last.

| Parameter | Meaning |
| --- | --- |
| `timeout` | Maximum motion time in milliseconds. A negative timeout disables the time limit; keep a finite one while testing. |
| `errorExit` | Allowed absolute target error. Usually inches for translation or degrees for turns. `-1` disables this check. |
| `velocityExit` | Allowed measured speed at completion: inches/s or degrees/s. `-1` disables this check. |
| `maxSpeed` | Output cap from 0 to 127, not RPM or inches/s. Available on distance, turn, point, and pose motions. |
| `minSpeed` | Minimum requested output magnitude. Start at 0. A nonzero value is useful for passing through a target. |
| `forwards` | `true` approaches with the front; `false` approaches with the rear. Available on point and pose motions. |
| `async` | Defaults to `false`: the call waits for completion. `true` lets the next line run while the movement continues. |

For `moveDistance`, turns, and point/pose moves, **all enabled completion checks must be satisfied** for a normal early finish. Timeout or cancellation can still end the move. If all completion checks are disabled, it runs until timeout/cancellation. Barrier motions have special exit behavior described below.

Turns and point/pose moves ignore `velocityExit` when `minSpeed` is nonzero. `moveDistance` does not. Start with `minSpeed = 0` until normal stopping works.

The supplied lateral profile enables `halfPlaneExit` and uses a 2.4" tolerance. For initial point/pose testing, explicitly set `.halfPlaneExit = false` so you can judge the ordinary position and speed exits.

### Drive a relative distance: `moveDistance`

```cpp
chassis.moveDistance(24, {.timeout = 2500, .maxSpeed = 90});
chassis.moveDistance(-12, {.timeout = 1500, .maxSpeed = 70});
```

The first moves forward 24" from where it starts. The second moves backward 12". It measures travel from the first vertical tracking sensor, which can be the drive-encoder fallback, and uses the correction controller to hold the stored heading.

Use this for straight segments when you care about distance from the current position. It is not a move to field Y = 24. It stops its drive output when it ends, even with nonzero `minSpeed`.

### Turn to a heading: `turnToHeading`

```cpp
chassis.turnToHeading(90, {.timeout = 1500, .maxSpeed = 90});
```

This faces the absolute heading 90°. Calling it twice does not turn another 90° the second time.

By default, `direction = gen::AngularDirection::AUTO` chooses the shortest turn. You can force a direction:

```cpp
chassis.turnToHeading(270, {
    .timeout = 2500,
    .direction = gen::AngularDirection::CW_CLOCKWISE,
    .maxSpeed = 80,
});
```

The other option is `gen::AngularDirection::CCW_COUNTERCLOCKWISE`. Give a forced long turn enough time; it may travel much farther than the shortest route. Use `AUTO` unless your route needs a particular direction.

### Face a coordinate: `turnToPoint`

```cpp
chassis.turnToPoint(24, 48, {.timeout = 1500});
chassis.turnToPoint(24, 48, {.timeout = 1500, .forwards = false});
```

The first faces the front toward `(24, 48)`. The second faces the rear toward it. The robot turns in place; it doesn't drive to that point. This is handy when lining up an intake or rear mechanism with a field object.

Use a point far enough away to define a useful direction. At the robot's own position, the direction to the point isn't meaningful.

### Swing turns: lock one side

A swing turn uses the same turn functions with `lockedSide`:

```cpp
chassis.turnToHeading(45, {
    .timeout = 2000,
    .lockedSide = gen::LockedSide::LEFT,
    .maxSpeed = 80,
});
```

`LEFT` commands the left side to zero and moves the right. `RIGHT` does the reverse. `NONE` is a normal two-sided turn. You can use this parameter with `turnToPoint` too.

A swing changes the robot center's position, so leave room for the arc. The stationary side is commanded to zero; how firmly it stays planted also depends on the drivetrain and brake behavior. Retest swing turns after tuning normal turns.

### Drive to a point: `moveToPoint`

```cpp
chassis.moveToPoint(24, 36, {
    .timeout = 3000,
    .velocityExit = 4.0f,
    .errorExit = 1.0f,
    .forwards = true,
    .maxSpeed = 90,
    .halfPlaneExit = false,
});
```

This steers toward a position while driving. It doesn't promise a final heading. Within 6" of the point, this implementation stops applying angular correction, so line up reasonably before a short or precise approach.

`settle = true` adjusts forward output near the target based on alignment. It is enabled by default. It is not a setting that adds a settling delay, and `settle = false` does not disable the normal exit checks.

For reverse travel, use `.forwards = false`; keep X and Y as the actual destination coordinates. Don't negate the coordinate just because you're driving backward.

### Drive to a position and approach heading: `moveToPose`

```cpp
chassis.moveToPose(24, 36, 90, {
    .timeout = 3500,
    .velocityExit = 4.0f,
    .errorExit = 1.0f,
    .forwards = true,
    .dLead = 8.0f,
    .gLead = 0.25f,
    .maxSpeed = 90,
    .halfPlaneExit = false,
});
```

The destination is `(24, 36)` with the robot's front intended to face 90°. It uses an intermediate target, or "carrot," to shape the approach, then steers toward the requested heading near the end.

| Extra parameter | What to do with it |
| --- | --- |
| `dLead` | Initial carrot distance behind the destination along the approach direction, in inches. Defaults to 0. A moderate positive value gives more room to line up; too much can create a wide detour. |
| `gLead` | Controls the extra ghost target between the initial and moving carrot. Defaults to 0, which makes it coincide with the moving carrot. Start at 0; try small values between 0 and 1 after the basic move works. |
| `chasePower` | Optional curve-speed limiter. `-1` disables it. Smaller positive values slow curved sections more; larger values allow more output. It is not a second `maxSpeed`. |

Start with `gLead = 0` and adjust `dLead` first. Add complexity only if the approach needs it.

For a backward approach, `.forwards = false` still interprets `theta` as the **robot's front heading**. For example, driving backward toward `(0, -24)` while the front faces +Y uses `theta = 0`.

The normal completion checks measure position and translational speed, not final heading error. If the next action needs precise orientation, follow the pose move with `turnToHeading()`.

### Cross a barrier: `crossBarrier`

```cpp
chassis.crossBarrier({
    .timeout = 2000,
    .errorExit = 3.0f,
    .speed = 70,
});
```

This drives at fixed output with heading correction while watching IMU **roll**. The current implementation must observe roll below −10° and above +10°, then finish within `errorExit` degrees of **−5° roll**. `velocityExit` is not used.

That makes it specific to the IMU orientation and obstacle shape it was written for. Check the roll readings on your robot; it isn't a generic "drive until level" command. Without a usable IMU or the expected roll changes, it runs until timeout. Don't use `waitUntil(distance)` with this motion; it doesn't update travel progress.

### Direct drive and the low-level turn step

`chassis.tank(left, right)`, `arcade(throttle, turn)`, and `curvature(throttle, turn)` directly command the drivetrain. They have no timeout and don't wait for a target. Stop them explicitly:

```cpp
chassis.tank(40, 40);
pros::delay(250);
chassis.tank(0, 0);
```

`turnToPointStep()` is also a low-level helper. It performs one PID update and writes output, without resetting the controller, waiting, applying a timeout, or stopping afterward. Its exit and async fields do not make it a complete motion. Use `turnToPoint()` for ordinary autons.

## 6. Write your first autonomous

Keep the first route simple enough that you can tell which step went wrong. Add a function inside the existing `namespace Auton` in `main.cpp`:

```cpp
void Practice() {
    chassis.setPose(0, 0, 0);

    chassis.moveDistance(24, {
        .timeout = 2500,
        .velocityExit = 4.0f,
        .errorExit = 1.0f,
        .maxSpeed = 80,
    });

    chassis.turnToHeading(90, {
        .timeout = 2000,
        .velocityExit = 3.0f,
        .errorExit = 2.0f,
        .maxSpeed = 80,
    });

    chassis.moveToPoint(24, 24, {
        .timeout = 2500,
        .velocityExit = 4.0f,
        .errorExit = 1.0f,
        .maxSpeed = 80,
        .halfPlaneExit = false,
    });

    chassis.tank(0, 0);
}
```

Starting at the origin facing +Y, this drives forward 24", faces +X, then drives toward `(24, 24)`. The calls block by default, so they run in order.

Test the first movement alone, then add the turn, then the last movement. Watch the pose at each stop. Once the route repeats reliably, add your intake, lift, or piston actions at the points where they belong.

### Add it to the included selector

GenSelector is already included and started by `initialize()`. You don't need to paste another selector into this template.

Add the routine to the existing list, after the function definitions:

```cpp
robot::AutonRoutineList autonRoutines = {
    {"Practice", Auton::Practice},
    {"Left", Auton::Left},
    {"Right", Auton::Right},
};
```

Each entry is a display name followed by a function with no arguments and a `void` return type. The first entry is selected initially. Choosing an entry doesn't run it; `autonomous()` runs the selected function when autonomous starts:

```cpp
void autonomous() {
    autonSelector.runSelected(Auton::Practice);
}
```

The argument is a fallback if the selected entry is unavailable. It doesn't override a valid selection.

In `autonSelectorConfig`, change:

- `.menu.teamNumber` to your team number.
- `.devices` to the motors you want on the three temperature gauges. Use index `0` for the first motor in a group; the gauge reads that motor, not the group average.
- `.terminal.fields` if you want different telemetry. The existing X/Y/Theta getter functions are already connected to the chassis.

A telemetry entry such as `{"X", selectorX, 2}` means label X, call `selectorX()`, show two decimal places. Pass the function name, not the result of calling it.

On the brain screen, tapping the highlighted row advances the selection. The faded row above goes back; the lower rows advance. Confirm the highlighted routine before enabling autonomous.

`BrainScreen` is the current input mode. The selector also supports a controller button, ADI digital input, and a custom new-press callback; their fields are in `src/GenSelector/selector.hpp` if you need a physical selector later.

The supplied `Left` and `Right` routines use robot-specific mechanisms and starting poses. Replace them as you build your own routes; they aren't generic left/right field autons.

## 7. Chaining and running mechanisms

### Let a mechanism run during a movement

You can often start a mechanism before a normal blocking movement and stop it afterward. Use async when you need an action partway through:

```cpp
// Inside an auton; assumes you've configured an intake motor group.
chassis.moveDistance(24, {
    .timeout = 2500,
    .maxSpeed = 80,
    .async = true,
});

chassis.waitUntil(10);
intake.move(127);
chassis.waitUntilDone();
intake.move(0);
```

`waitUntil(10)` waits for the active motion's progress to pass 10, **or for the motion to finish**. For translation that progress is inches traveled; for turn motions it is accumulated degrees turned. It is not remaining distance to the target. A timeout can release the wait before the desired distance is reached, so don't treat it as proof of arrival.

`waitUntilDone()` waits for the current motion to end. If you call it immediately after a normal blocking motion, that motion has already finished.

Use one async chassis movement at a time and wait before launching the next. The current motion bookkeeping isn't a general-purpose queue for lots of concurrent movement tasks. Don't send driver/direct-drive commands while an async movement is also controlling the chassis.

### Pass through a point without stopping

For point and pose motions, nonzero `minSpeed` lets the previous motor command remain active when the movement returns normally, including when it times out. You must hand off immediately to the next movement or stop explicitly.

```cpp
chassis.moveToPoint(0, 24, {
    .timeout = 2000,
    .velocityExit = -1,
    .errorExit = -1,
    .maxSpeed = 90,
    .minSpeed = 25,
    .halfPlaneExit = true,
    .halfPlaneTolerance = 1.0f,
    .settle = false,
});

// No delay here: the previous move can still be commanding the motors.
chassis.moveToPoint(0, 48, {
    .timeout = 2500,
    .velocityExit = 4.0f,
    .errorExit = 1.0f,
    .maxSpeed = 90,
    .minSpeed = 0,
    .halfPlaneExit = false,
    .settle = true,
});
```

A half-plane exit checks whether the robot crosses an approach boundary near the target. A positive tolerance moves the boundary before the exact target, allowing an earlier handoff. `moveToPose` uses the requested approach heading for that boundary; `moveToPoint` uses the current travel-facing heading, so its boundary can rotate as it steers.

If other completion checks remain enabled, crossing the boundary alone isn't enough: those checks must be satisfied too. That's why the first move above disables error and velocity checks. For a final stop, use `minSpeed = 0` and normal position/speed exits.

`settle = false` changes near-target drive behavior, but **nonzero `minSpeed`** is what preserves output after a point/pose motion. Turning and `moveDistance` motions still command a stop at their ends.

Use `cancelMotion()` to cancel the active movement, or `cancelAllMotions()` to clear the active/queued motion flags. Both command the drivetrain to stop. Use these before handing control to a different driver or task.

## 8. Troubleshooting

| What you're seeing | What to check first |
| --- | --- |
| It spins when told to drive forward | Motor reversal and which group is left/right |
| A distance move is consistently too long or short | Wheel diameter, cartridge RPM, external gearing, and `wheelRpm` |
| X/Y barely reflect the physical movement | Sensor connections, active odometry profile, and whether calibration ran |
| X goes the wrong way during a sideways test | Horizontal sensor direction; raw horizontal distance is positive toward the robot's left |
| X/Y drift during an in-place turn | Tracking-wheel directions and signed offsets |
| Straight distance moves veer | Mechanical drag, motor configuration, and the currently zero `correctionGains` |
| It shoots past the target | P too high, D too low, excessive I, or too much `minSpeed` |
| It stops short and immediately starts the next step | Loose exits or a short timeout; inspect these before raising gains |
| Every move uses its whole timeout | Disabled exits, unreachable tolerances, bad odometry, or an unsatisfied half-plane check |
| Small turns work but large ones don't | Check the fitted curve, output saturation, and the recorded gains across different sizes |
| Turns work but point/pose moves don't | Those motions sample P at fixed inputs: lateral 12 and angular 180 |
| It keeps driving after a point/pose move returns | Nonzero `minSpeed` preserves output; start the next move immediately or command a stop |
| A barrier move always times out | IMU roll doesn't follow the expected negative/positive sequence and final −5° region |
| Named parameters fail to compile | Check spelling, parameter type, and declaration order in `chassis.hpp` |
| Selector/LVGL symbols fail to compile or link | Keep the matching LVGL headers/library and all selector source/image files together |

Once the basic route works, run it several times without changing anything. Then test with the mechanisms carrying their expected load. A tune that repeats is more useful than one very fast run that only works occasionally.

## 9. GenSelector setup and reference

GenSelector is already set up in this template. If you're staying in GenClient, use the routine list and config described in [the autonomous section](#add-it-to-the-included-selector); you don't need to paste another copy of the setup code.

The original GenSelector guide is included below for reference and for copying the selector into another PROS project. Its full `main.cpp` example is a standalone selector example. When adding it to an existing robot project, merge the includes, config, and lifecycle calls into your existing code. Keep your chassis calibration, autonomous routines, and driver control.

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

### Setup First

Before copying any code, make sure your project has the same base tools this selector was built against.

#### Required Versions

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

#### If You Do Not Have PROS Installed

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

#### If You Already Have PROS But Not LVGL

GenSelector requires the PROS `liblvgl` template.

Check your project first:

- if you already have `include/liblvgl/`, LVGL is already present
- if `include/liblvgl/` is missing, add/install the `liblvgl` template before using GenSelector

Practical rule:

- if you are unsure, create a fresh PROS 4 V5 project with LVGL included, then copy `src/GenSelector/` into that project

#### Recommended Baseline

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

### What It Does

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

### What To Copy

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

### What To Paste In `main.cpp`

You need to paste five things into `src/main.cpp`:

1. The include
2. Your motors and telemetry getters
3. Your autonomous routine list
4. The selector config and selector object
5. The lifecycle hooks in `initialize()` and `autonomous()`

---

### 1. Paste The Include

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

### 2. Paste Your Motors And Telemetry Getters

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

### 3. Paste Your Autonomous Functions And List

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

### 4. Paste The Selector Config And Object

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

#### What `.devices` means

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

#### What `.terminal.fields` means

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

#### Input modes

For brain screen touch, use:

```cpp
.input = {
    .type = robot::SelectorInputType::BrainScreen,
},
```

That is the default setup for the current project.

---

### 5. Paste The PROS Lifecycle Hooks

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

### Full Example `main.cpp`

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

### Common Mistakes

#### 1. Putting the folder in `include/`

Do not put the whole selector folder in `include/`.

Use:

```txt
src/GenSelector/
```

Reason:

- `selector.cpp` and the `.c` image files need to be compiled as source files

#### 2. Using direct values in `.fields`

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

#### 3. Expecting LVGL `%f` formatting to work

This project already avoids that internally.

If you see `f` on screen, it means some other part of your project is still using float formatting through an embedded `printf` path that does not support it.

#### 4. Wrong asset symbol names

The image files need to expose:

- `background`
- `logosmall`

If those names change, the selector will not link correctly.

---

### Files You Usually Edit

- `src/main.cpp`
- `src/GenSelector/selector.cpp`
- `src/GenSelector/selector.hpp`

Most users only need to edit `main.cpp`.

---

### GenClient

GenSelector is intended to be used alongside GenClient.

Typical pattern:

- GenClient handles chassis, odom, motion, and robot systems
- GenSelector handles autonomous selection and brain-screen display

If you already have GenClient running, replace the example telemetry getters with your real chassis pose getters and replace the example auton vector with your real routines.

Questions or contributions: message `nickson78181a` on Discord.
THANK YOU DANIEL GENESIS FOR ALL THE ALGS (ALL CREDITS TO THE GOAT)
