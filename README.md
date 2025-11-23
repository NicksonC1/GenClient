# VEX PROS Template (gen-client)

A ready-to-use VEX V5 PROS template with common subsystems, driver utilities, and autonomous helpers baked in. Configure your robot, drop in your auton routines, and start iterating fast.

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

## Project Layout
- `src/main.cpp`: Entry point; wires up controllers, sensors, pistons, and starts the auton selector.
- `include/gen/electronics.h`: Motor, piston, controller helpers and limit switch namespace.
- `include/gen/auton.h`: Auton routine stubs and the routine vector to customize.
- `include/gen/selector.h` / `src/gen/selector.cpp`: Auton selector task and LCD display logic.
- `include/gen/motion.h` and related files: Motion/odometry and PID primitives.
- `src/gen/colorsort.cpp`: Color sort scaffolding (extend as needed).

## Getting Started
1) Configure ports in `src/main.cpp` (motors, sensors, pistons) and adjust drive mode/deadband/curve.
2) Fill out auton functions and `routines` in `include/gen/auton.h`.
3) Build and upload with PROS (`pros build`, `pros upload`), then select auton via the limit switch and LCD.

## How to Extend
- Add more subsystems by following the patterns in `electronics.h` (wrapping PROS devices with small helpers).
- Drop new motion primitives into `motion`/`pid` modules if you need custom profiles or constraints.
- Expand the auton selector inputs (e.g., partner controller button) by editing `src/gen/selector.cpp`.

## Notes
- Uses PROS APIs; ensure your toolchain is installed and firmware is up to date.
- Drive curves and deadbands are configurable at runtime; see `gen::Controller` setters.
- Odometry supports multiple tracking geometries—pick the setup that matches your hardware. 
