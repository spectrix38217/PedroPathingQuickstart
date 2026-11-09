# AGENTS.md — Repository Architecture & Engineering Specifications

This document records the established architectural patterns, hardware configurations, control models, and code standards for the QuickStart FTC repository. It serves as persistent context for all future AI agents and developers working on this codebase.

---

## 1. Project Overview & Control Architecture

### Target Environment & Core Dependencies
- **FTC SDK Version**: `12.0.0`
- **Android Gradle Plugin (AGP)**: `8.13.2` (Java 8 compatibility, `minSdkVersion 24`, `compileSdk 34`)
- **Pedro Pathing Stack**:
  - `com.pedropathing:revhub:3.0.1` (RevHub Drivetrains & Localizers)
  - `com.pedropathing:tuning:1.0.1` (Autotuning Procedures)

### Control Model & Execution Architecture
1. **Localization & Path Following**:
   - Driven by Pedro Pathing's `Follower` class using `OctoQuadLocalizer` (OctoQuad hardware encoder interface).
   - All spatial calculations operate in **inches** for position $(X, Y)$ and **radians** internally for heading ($\theta$), with telemetry formatted in **degrees** ($^\circ$).
2. **TeleOp Control Architecture**:
   - **Field-Centric Driving**: Driver stick inputs are rotated into the field frame using `ManualDrive.fieldCentric(...)` based on `follower.pose().heading()`.
   - **Absolute Pointing Steering System**: The right stick controls the target robot heading based on the vector angle it is pointed at (`Math.atan2(-rx, ry)`). Direct Up = 0°, Right = 90°, Down = 180°, Left = 270° (-90°), matching all 360° vector directions continuously with field-aligned X component inversion. High derivative damping (`ABSOLUTE_HEADING_D = 0.08`, `ABSOLUTE_HEADING_P = 0.45`, `MAX_ABSOLUTE_TURN_POWER = 0.50`) ensures smooth, non-oscillating rotation that settles on target with zero overshoot. Releasing the stick holds the last pointed heading via the damped PD controller.
   - **Stationary Position & Heading Lock (Defense Resistance)**: When the driver is not steering (`|left_stick_y| <= 0.05`, `|left_stick_x| <= 0.05`, right stick magnitude `< 0.20`), a 350ms settling window (`SETTLING_DELAY_SEC = 0.35`) allows heavy rotational and translational momentum to coast down naturally before engaging `follower.hold(targetPositionPose)` to hold both position $(X, Y)$ and heading ($\theta$) against defense/pushing.
   - **Subsystem Encapsulation & State Machine**: Non-drivetrain mechanism hardware is encapsulated in `BallMechanism.java`, governed by an internal state machine (`IDLE`, `INTAKE`, `OUTTAKE`, `SHOOTING`).
   - **Intake Toggle State**: `X` button toggles `INTAKE` mode ON and OFF. Driving the robot does not affect intake state. Outtaking (`L1`) or Shooting (`R1`) interrupts and turns off intake mode.
   - **Low-Pass Filtered Shooter PI Control**: High-speed flywheel velocity is monitored via encoder feedback (`shooter.getVelocity()`) in `RUN_WITHOUT_ENCODER` mode to avoid REV Control Hub firmware PID conflicts. Raw RPM is smoothed using an exponential low-pass filter ($\alpha = 0.25$) and maintained using a Feedforward + PI (Proportional-Integral) error correction algorithm with anti-windup clamping (`updateShooterSpeed()`).
   - **Shooter Speed Adjustment**: `adjustShooterRpm(±100.0)` dynamically tunes target RPM (target shooting RPM when shooting, or target idle RPM when idle).
   - **Shooter Ramp-Up & Continuous Feed Shooting Strategy**: When shooting, the shooter flywheel ramps to 4750 RPM default (adjustable via `L2 + D-Pad Left/Right`). The feeder servo and transfer motor wait until the flywheel reaches target velocity within a tight **50 RPM** buffer (`SHOOTER_RPM_TOLERANCE = 50.0`) OR a 1.0s timeout expires. Once ready, the stopper servo OPENS ($0.5$), and both feeder servo and transfer motor run continuously to feed the shooter. Maximum corrective boost power above feedforward is capped (`SHOOTER_MAX_BOOST_POWER = 0.20`) with instant integral reset on overspeed to prevent aggressive re-ramp overshooting after a ball is shot.
   - **Position Preservation & Handoff**: `OpModeStorage.autonomousEndPose` and `SpectrixTeleOp.startingPose` preserve the robot's field pose seamlessly between Autonomous and TeleOp.
3. **Autonomous Architecture**:
   - Standard Sequential `LinearOpMode` (`SpectrixAutoTest`).
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
  - `barLeft` — Servo (Hardware map name: `"barLeft"`, neutral: `0.5`, offset: `-0.45` = `0.05`)
  - `barRight` — Servo (Hardware map name: `"barRight"`, neutral: `0.5`, offset: `+0.45` = `0.95`)
- **Stopper Servo**:
  - `stopper` — Servo (Hardware map name: `"stopper"`, open: `0.5`, closed: `0.22` by default; opens automatically when shooter flywheel is at speed during `SHOOTING` and closes immediately upon release)
- **Feeder, Transfer & Shooter**:
  - `feeder` — Continuous Rotation Servo (Hardware map name: `"feeder"`, shoot power: `-1.0`, hold: `0.0`)
  - `transfer` — Motor (Hardware map name: `"transfer"`, intake power: `0.8`, outtake: `-0.6`, zero power behavior: `BRAKE`)
- **Shooter Flywheel**:
  - `shooter` — `DcMotorEx` with encoder feedback (Hardware map name: `"shooter"`, max physical RPM at 12V: `5000.0` RPM, idle speed: `2500.0` RPM, shooting speed: `4750.0` RPM, zero power behavior: `FLOAT`)

---

## 3. Code Standards & Architecture Guidelines

### Subsystem Design Rules
- **Centralized Constants**: All hardware map names, servo positions, motor power levels, and RPM targets MUST be defined in `Constants.java` (`Constants.Servos`, `Constants.Motors`).
- **Encapsulation**: Non-drivetrain hardware MUST be encapsulated inside dedicated subsystem classes (e.g., `BallMechanism.java`).
- **Initialization**: Drive and localizers must be instantiated via `Constants.create(hardwareMap)`. Subsystem classes must accept `HardwareMap` in their constructor.
- **Mandatory Subsystem Updates**: Subsystem state machines and velocity loops (`ballMechanism.update()`) MUST be called once per loop iteration in `loop()`.
- **Telemetry Layout Order Standards**:
  - `build`: Always first line (`BuildConfig.BUILD_TIME`).
  - `position`: `X: %.2f in, Y: %.2f in, H: %.2f°`.
  - `shooterRPM`: Prominent top parameter `%.0f / %.0f RPM [ShootTarget: %.0f RPM] (Pwr: %.2f)`.
  - `state`: Prominent top parameter `Mech: %s | AutoDrive: %s`.

### Loop Safety & Non-Blocking Rules
- **Strict Non-Blocking TeleOp**: `loop()` MUST be non-blocking. Never call `Thread.sleep()` or blocking `while` loops inside TeleOp.
- **Mandatory `follower.update()`**: `follower.update()` MUST be called once per loop iteration in all OpModes to maintain localizer tracking and motor control outputs.

### TeleOp Control Scheme & Button Mapping Standard
| Input / Gesture | Function / Behavior |
| :--- | :--- |
| **Left Stick Y/X** | Field-centric Translational Drive (Forward/Backward, Strafe Left/Right) |
| **Right Stick Vector (X/Y)** | **Absolute Pointing Steering**: Robot heading matches the exact 360° angle pointed at by the right stick (Up = 0°, Right = 90°, Down = 180°, Left = 270°); holds last pointed heading on release. |
| **Left Trigger (`L2`)** | Analog Slow Driving Speed (scales max speed down to `slowModeMultiplier`) |
| **Left Trigger (`L2`) + D-Pad Up / Down** | Adjusts `slowModeMultiplier` in steps of `0.1` (clamped between `0.1` and `1.0`) |
| **Left Trigger (`L2`) + D-Pad Left / Right** | Adjusts target shooter RPM down/up in steps of `100` RPM (modifies shooting RPM when shooting, or idle RPM when idle) |
| **Y Button (without D-Pad)** | BallMechanism: Toggles flower release bar Up (`barUp()`, $0.5$) or Down (`barDown()`, $0.5 \pm 0.45$) |
| **X Button (without D-Pad)** | BallMechanism: **Toggles** `INTAKE` mode (Forward / $80\%$) ON <-> OFF |
| **Left Bumper (`L1`) + X Button** | BallMechanism: **Toggles** `INTAKE_REVERSE` mode (Reverse / $-60\%$) ON <-> OFF, or reverses direction if already intaking forward |
| **Left Bumper (`L1`) (without X or D-Pad)** | BallMechanism: State transition to `OUTTAKE` (`transfer` motor backwards $60\%$ / `-0.6`); turns intake OFF when released |
| **Right Bumper (`R1`) (without D-Pad)** | BallMechanism: State transition to `SHOOTING` (shooter @ 4700 RPM default, stopper OPEN $0.5$ when at speed, `feeder` CRServo reverse $-1.0$, `transfer` motor continuous $80\%$ / `0.8`); turns intake OFF when released |
| **Y + D-Pad Up / Down / Left / Right** | Saves current robot pose and heading to Slot 1 (Up), Slot 2 (Down), Slot 3 (Left), or Slot 4 (Right) |
| **A + D-Pad Up / Down / Left / Right** | Recalls and drives autonomously to user-saved pose Slot 1 (Up), Slot 2 (Down), Slot 3 (Left), or Slot 4 (Right) |
| **Right Bumper (`R1`) + D-Pad Up / Down / Left / Right** | Recalls pre-coded preset positions: Parking Blue (Up: 26.68, -49.84, 90°), Parking Red (Down: 99.27, 74.51, 270°), Shooting Area Left (Left: 17.21, 10.67, 360°), Shooting Area Right (Right: 113.19, 9.84, 180°) |
| **Left Bumper + Right Bumper + X** | Recalibrates robot position to origin `(0, 0, 0°)` |
| **B Button** | Aborts automated return-to-position drive |

---


## 4. Canonical Reference Examples

The following files represent the **golden reference implementations** for this repository:

1. **TeleOp Implementation**:
   - [SpectrixTeleOp.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixTeleOp.java) — Golden reference for Field-Centric TeleOp, Position & Heading Lock, Analog Slow Mode, Position Save/Return, and Subsystem integration.

2. **Mechanism Subsystem Implementation**:
   - [BallMechanism.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BallMechanism.java) — Golden reference for non-drivetrain mechanism encapsulation (`barLeft`, `barRight`, `stopper`, `feeder`, `transfer`, `shooter`).

3. **Standard Autonomous Implementation**:
   - [SpectrixAuto.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixAuto.java) — Golden reference for production `LinearOpMode` autonomous execution with Pedro Pathing and OpModeStorage pose handoff.
   - [SpectrixAutoTest.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixAutoTest.java) — Golden reference for sample procedural execution with Pedro Pathing.

4. **Servo Calibration Implementation**:
   - [BarCalibTeleOp.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BarCalibTeleOp.java) — Golden reference for tuning and calibrating servo positions.

5. **Hardware & Localizer Configuration**:
   - [Constants.java](file:///C:/Users/spectrix/StudioProjects/PedroPathingQuickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java) — Golden reference for Mecanum drivetrain, OctoQuad localizer, Foresight algorithm settings, and mechanism constants.

---

## 5. External Reference Code (`examples/` Directory)

- **Purpose**: The `examples/` folder in the repository root (e.g. `examples/MOEBo-Sapiens-Decode/`) contains sample code from other teams for conceptual inspiration.
- **Usage Rule for Agents & Developers**: Use these files for **high-level architectural ideas and logic ONLY** (e.g., state machine transitions, sensor rumble feedback). **DO NOT copy code directly** from `examples/`, as it may use older API packages or non-standard conventions. Always adapt concepts to match the active Pedro Pathing 1.1.1 architecture, `BallMechanism` state machine, and code standards defined in this document.
