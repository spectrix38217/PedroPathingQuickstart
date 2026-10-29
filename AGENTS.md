# AGENTS.md — Repository Architecture & Engineering Specifications

This document records the established architectural patterns, hardware configurations, control models, and code standards for the QuickStart FTC repository. It serves as persistent context for all future AI agents and developers working on this codebase.

---

## 1. Project Overview & Control Architecture

### Target Environment & Core Dependencies
- **FTC SDK Version**: `12.0.0`
- **Android Gradle Plugin (AGP)**: `8.13.2` (Java 8 compatibility, `minSdkVersion 24`, `compileSdk 34`)
- **Pedro Pathing Stack**:
  - `com.pedropathing.ivy:pedro:1.1.1` (Ivy Command-Based Framework)
  - `com.pedropathing:revhub:3.0.1` (RevHub Drivetrains & Localizers)
  - `com.pedropathing:tuning:1.0.1` (Autotuning Procedures)

### Control Model & Execution Architecture
1. **Localization & Path Following**:
   - Driven by Pedro Pathing's `Follower` class using GoBilda Pinpoint localizer sensor fusion.
   - All spatial calculations operate in **inches** for position $(X, Y)$ and **radians** internally for heading ($\theta$), with telemetry formatted in **degrees** ($^\circ$).
2. **TeleOp Control Architecture**:
   - **Field-Centric Driving**: Driver stick inputs are rotated into the field frame using `ManualDrive.fieldCentric(...)` based on `follower.pose().heading()`.
   - **Damped PD Heading Lock**: When the driver is not turning (`|right_stick_x| <= 0.05`), a damped PD controller holds the robot's heading without drift or oscillation.
   - **Position Preservation & Handoff**: `OpModeStorage.autonomousEndPose` and `SpectrixTeleOp.startingPose` preserve the robot's field pose seamlessly between Autonomous and TeleOp.
3. **Autonomous Architecture**:
   - Supports both **Standard Sequential `LinearOpMode`** and **Command-Based `Ivy` Framework** (`Scheduler`, `Command.build()`, `sequential`, `race`).
   - Max path speed is capped at **50%** (`maxPathSpeed = 0.5`) across all autonomous routines to eliminate overshooting.

---

## 2. Hardware Configuration & Mappings

### Drivetrain Configuration
- **Drive Type**: 4-wheel Mecanum Drivetrain
- **Hardware Map Names**:
  - `frontLeft` — Motor (Direction: `REVERSE`)
  - `frontRight` — Motor (Direction: `FORWARD`)
  - `backLeft` — Motor (Direction: `REVERSE`)
  - `backRight` — Motor (Direction: `FORWARD`)
- **Zero-Power Behavior**: Automatically handled by `Mecanum` drivetrain class (`FLOAT` during manual drive, `BRAKE` when stopped/holding).

### Localizer Configuration (Pinpoint)
- **Hardware Map Name**: `pinpoint`
- **Driver**: `GoBildaPinpointDriver` via `PinpointLocalizer`
- **Pod Type**: `goBILDA_4_BAR_POD` (32mm omni wheels)
- **Pod Offsets**:
  - `xPodOffset`: `6.203199371578187` inches
  - `yPodOffset`: `-0.017317104058002863` inches
  - `offsetUnits`: `DistanceUnit.INCH`
- **Encoder Directions**:
  - `xPodDirection`: `EncoderDirection.REVERSED`
  - `yPodDirection`: `EncoderDirection.FORWARD`
- **Global Distance Unit**: `DistanceUnit.INCH`

---

## 3. Code Standards & Architecture Guidelines

### Subsystem Design Rules
- **Initialization**: Hardware and localizers must be instantiated via `Constants.create(hardwareMap)`.
- **Encapsulation**: State variables must be private to the OpMode class and initialized cleanly in `init()`.
- **Telemetry Standards**:
  - `position`: Formatted with 2 decimal precision and degrees: `X: %.2f, Y: %.2f, H: %.2f°`.
  - `velocity`: Formatted with 2 decimal precision and degrees/sec: `VX: %.2f, VY: %.2f, W: %.2f°/s`.
  - `build`: Always include `BuildConfig.BUILD_TIME` on telemetry.

### Loop Safety & Non-Blocking Rules
- **Strict Non-Blocking TeleOp**: `loop()` MUST be non-blocking. Never call `Thread.sleep()` or blocking `while` loops inside TeleOp.
- **Mandatory `follower.update()`**: `follower.update()` MUST be called once per loop iteration in all OpModes to maintain localizer tracking and motor control outputs.
- **Ivy Scheduler Loop**: When using the Ivy framework, `Scheduler.execute()` and `follower.update()` MUST run every iteration inside `while (opModeIsActive())`.
- **Mandatory `Scheduler.reset()`**: Every Ivy autonomous OpMode MUST invoke `Scheduler.reset()` at the start of `runOpMode()` to clear lingering commands from previous runs.

### TeleOp Control Scheme & Button Mapping Standard
| Input / Gesture | Function / Behavior |
| :--- | :--- |
| **Left Stick Y/X** | Field-centric Translational Drive (Forward/Backward, Strafe Left/Right) |
| **Right Stick X** | Manual Heading Rotation (resets target heading on release after 250ms settling window) |
| **Left Trigger (`L2`)** | Analog Slow Driving Speed (scales max speed down to `slowModeMultiplier`) |
| **Left Trigger (`L2`) + D-Pad Up / Down** | Adjusts `slowModeMultiplier` in steps of `0.1` (clamped between `0.1` and `1.0`) |
| **Y + D-Pad Up / Down / Left / Right** | Saves current robot pose and heading to Slot 1 (Up), Slot 2 (Down), Slot 3 (Left), or Slot 4 (Right) |
| **A + D-Pad Up / Down / Left / Right** | Recalls and drives autonomously to user-saved pose Slot 1 (Up), Slot 2 (Down), Slot 3 (Left), or Slot 4 (Right) |
| **Right Bumper (`R1`) + D-Pad Up / Down / Left / Right** | Recalls pre-coded preset positions: Parking Blue (Up: 26.68, -49.84, 90°), Parking Red (Down: 99.27, 74.51, 270°), Shooting Area Left (Left: 17.21, 10.67, 360°), Shooting Area Right (Right: 113.19, 9.84, 180°) |
| **Left Bumper + Right Bumper + X** | Recalibrates robot position to origin `(0, 0, 0°)` |
| **B Button** | Aborts automated return-to-position drive |
---

## 4. Canonical Reference Examples

The following files represent the **golden reference implementations** for this repository:

1. **TeleOp Implementation**:
   - [SpectrixTeleOp.java](file:///C:/Users/lajos/Projects/QuickStart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixTeleOp.java) — Golden reference for Field-Centric TeleOp, Damped PD Heading Lock, Analog Slow Mode, and Position Save/Return.

2. **Standard Autonomous Implementation**:
   - [SpectrixAutoTest.java](file:///C:/Users/lajos/Projects/QuickStart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixAutoTest.java) — Golden reference for procedural `LinearOpMode` autonomous execution with Pedro Pathing.

3. **Ivy Command-Based Autonomous Implementation**:
   - [SpectrixAutoIvyTest.java](file:///C:/Users/lajos/Projects/QuickStart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixAutoIvyTest.java) — Golden reference for Ivy Command-Based autonomous routines (`Scheduler`, `Command.build()`, `sequential`, `race`).

4. **Hardware & Localizer Configuration**:
   - [Constants.java](file:///C:/Users/lajos/Projects/QuickStart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java) — Golden reference for Mecanum drivetrain, GoBilda Pinpoint localizer, and Foresight algorithm settings.
