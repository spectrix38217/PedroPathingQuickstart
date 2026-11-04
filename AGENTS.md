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
   - Driven by Pedro Pathing's `Follower` class using `OctoQuadLocalizer` (OctoQuad hardware encoder interface).
   - All spatial calculations operate in **inches** for position $(X, Y)$ and **radians** internally for heading ($\theta$), with telemetry formatted in **degrees** ($^\circ$).
2. **TeleOp Control Architecture**:
   - **Field-Centric Driving**: Driver stick inputs are rotated into the field frame using `ManualDrive.fieldCentric(...)` based on `follower.pose().heading()`.
   - **Damped PD Heading Lock**: When the driver is not turning (`|right_stick_x| <= 0.05`), a damped PD controller holds the robot's heading without drift or oscillation (includes 250ms settling window).
   - **Subsystem Encapsulation**: Non-drivetrain mechanism hardware is encapsulated in `BallMechanism.java`.
   - **Position Preservation & Handoff**: `OpModeStorage.autonomousEndPose` and `SpectrixTeleOp.startingPose` preserve the robot's field pose seamlessly between Autonomous and TeleOp.
3. **Autonomous Architecture**:
   - Supports both **Standard Sequential `LinearOpMode`** and **Command-Based `Ivy` Framework** (`Scheduler`, `Command.build()`, `sequential`, `race`).
   - Capped path speeds prevent overshooting during automated positioning.

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

### Localizer Configuration (OctoQuad)
- **Hardware Map Name**: `octoquad`
- **Driver**: `OctoQuadLocalizer` via Digital Chicken Labs OctoQuad hardware interface
- **Encoder Ports**:
  - `xPodPort`: `0`
  - `yPodPort`: `1`
- **Pod Offsets & Scalar**:
  - `ticksPerUnit`: `505.316944406` ticks/inch
  - `xPodOffset`: `6.181102362204725` inches
  - `yPodOffset`: `1.5748031496062993` inches
  - `headingScalar`: `1.0095375423565784`
  - `offsetUnits` & `globalDistanceUnit`: `DistanceUnit.INCH`

### Mechanism Hardware Mappings (`BallMechanism`)
- **Flower Release Servos**:
  - `barLeft` — Servo (Hardware map name: `"barLeft"`, neutral: `0.5`, offset: `-0.25` = `0.25`)
  - `barRight` — Servo (Hardware map name: `"barRight"`, neutral: `0.5`, offset: `+0.25` = `0.75`)
- **Feeder & Transfer**:
  - `feeder` — Continuous Rotation Servo (Hardware map name: `"feeder"`, shoot power: `-1.0`, hold: `0.0`)
  - `transfer` — Motor (Hardware map name: `"transfer"`, intake power: `1.0`, outtake: `-1.0`, zero power behavior: `BRAKE`)

---

## 3. Code Standards & Architecture Guidelines

### Subsystem Design Rules
- **Centralized Constants**: All hardware map names, servo positions, and motor power levels MUST be defined in `Constants.java` (`Constants.Servos`, `Constants.Motors`).
- **Encapsulation**: Non-drivetrain hardware MUST be encapsulated inside dedicated subsystem classes (e.g., `BallMechanism.java`).
- **Initialization**: Drive and localizers must be instantiated via `Constants.create(hardwareMap)`. Subsystem classes must accept `HardwareMap` in their constructor.
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
| **Y Button (without D-Pad)** | BallMechanism: Toggles flower release bar Up (`barUp()`, $0.5$) or Down (`barDown()`, $0.5 \pm 0.25$) |
| **X Button (without D-Pad)** | BallMechanism: Intakes balls (`intake()`, `transfer` motor at $80\%$ / `0.8`, `feeder` holding $0.0$) |
| **Left Bumper (`L1`) (without D-Pad)** | BallMechanism: Outtakes balls (`outtake()`, `transfer` motor backwards $60\%$ / `-0.6`, `feeder` holding $0.0$) |
| **Right Bumper (`R1`) (without D-Pad)** | BallMechanism: Transfer to shoot (`transferToShoot()`, `transfer` motor at $80\%$ / `0.8`, `feeder` CRServo reverse $-1.0$) |
| **Y + D-Pad Up / Down / Left / Right** | Saves current robot pose and heading to Slot 1 (Up), Slot 2 (Down), Slot 3 (Left), or Slot 4 (Right) |
| **A + D-Pad Up / Down / Left / Right** | Recalls and drives autonomously to user-saved pose Slot 1 (Up), Slot 2 (Down), Slot 3 (Left), or Slot 4 (Right) |
| **Right Bumper (`R1`) + D-Pad Up / Down / Left / Right** | Recalls pre-coded preset positions: Parking Blue (Up: 26.68, -49.84, 90°), Parking Red (Down: 99.27, 74.51, 270°), Shooting Area Left (Left: 17.21, 10.67, 360°), Shooting Area Right (Right: 113.19, 9.84, 180°) |
| **Left Bumper + Right Bumper + X** | Recalibrates robot position to origin `(0, 0, 0°)` |
| **B Button** | Aborts automated return-to-position drive |

---

## 4. Canonical Reference Examples

The following files represent the **golden reference implementations** for this repository:

1. **TeleOp Implementation**:
   - [SpectrixTeleOp.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixTeleOp.java) — Golden reference for Field-Centric TeleOp, Damped PD Heading Lock, Analog Slow Mode, Position Save/Return, and Subsystem integration.

2. **Mechanism Subsystem Implementation**:
   - [BallMechanism.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BallMechanism.java) — Golden reference for non-drivetrain mechanism encapsulation (`barLeft`, `barRight`, `feeder`, `transfer`).

3. **Standard Autonomous Implementation**:
   - [SpectrixAutoTest.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixAutoTest.java) — Golden reference for procedural `LinearOpMode` autonomous execution with Pedro Pathing.

4. **Ivy Command-Based Autonomous Implementation**:
   - [SpectrixAutoIvyTest.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixAutoIvyTest.java) — Golden reference for Ivy Command-Based autonomous routines (`Scheduler`, `Command.build()`, `sequential`, `race`).

5. **Hardware & Localizer Configuration**:
   - [Constants.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java) — Golden reference for Mecanum drivetrain, OctoQuad localizer, Foresight algorithm settings, and mechanism constants.
