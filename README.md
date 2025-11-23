# GenClient

A ready-to-use VEX V5 PROS template with common subsystems, driver utilities, and autonomous helpers baked in. 
Configure your robot, drop in your auton routines, and start cooking up some autons!

## Features
- **Driver Control**
  - Multiple drive modes (tank 2-stick, arcade 1-stick, arcade 2-stick) with deadband and drive curve support.
  - Button helpers (`holding`, `pressing`) with a clean enum for controller inputs.
- **Electronics Helpers**
  - MotorGroup wrapper for RPM/gear-ratio control, percent-based moves.
  - Piston wrapper with state tracking and toggle helpers.
  - Shared limit switch namespace for ADI inputs.
- **Autonomous Selector**
  - Reusable selector module (`gen::selector`) driven by a limit switch; displays selection on the controller LCD.
  - Auton routine registry lives in `include/gen/auton.h`—edit the stub functions and the `routines` vector.
- **Motion/Control**
  - PID utilities and motion helpers (move distance, move to point/pose, turn to point/heading).
  - Odometry support for multiple tracking setups (zero tracker, 1/2 trackers, holonomic options).
  - Color sorting scaffolding for optical sensor workflows.
- **Color Sort**
- **Distance Reset**

## Questions:
- Message 'nickson78181a' on discord if you have any questions or want to contribute.
