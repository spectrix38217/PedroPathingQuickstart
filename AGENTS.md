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
   - **Manual Turning Scale**: Right stick manual rotation input is scaled to a maximum of **60%** (`MAX_MANUAL_TURN_SCALE = 0.60`) for responsive, controlled turning.
   - **Stationary Position & Heading Lock (Defense Resistance)**: When the driver is not steering (`|left_stick_y| <= 0.05`, `|left_stick_x| <= 0.05`, `|right_stick_x| <= 0.05`), a 350ms settling window (`SETTLING_DELAY_SEC = 0.35`) allows heavy rotational and translational momentum to coast down naturally before engaging `follower.hold(targetPositionPose)` to hold both position $(X, Y)$ and heading ($\theta$) against defense/pushing.
   - **Subsystem Encapsulation & State Machine**: Non-drivetrain mechanism hardware is encapsulated in `BallMechanism.java`, governed by an internal state machine (`IDLE`, `INTAKE`, `OUTTAKE`, `SHOOTING`).
   - **Intake Toggle State**: `X` button toggles `INTAKE` mode ON and OFF. Driving the robot does not affect intake state. Outtaking (`L1`) or Shooting (`R1`) interrupts and turns off intake mode.
   - **Low-Pass Filtered Shooter Control**: High-speed flywheel velocity is monitored via encoder feedback (`shooter.getVelocity()`), smoothed using an exponential low-pass filter ($\alpha = 0.25$), and maintained using a Feedforward + Capped Proportional Error correction algorithm (`updateShooterSpeed()`).
   - **Shooter Speed Adjustment**: `adjustShooterRpm(±100.0)` dynamically tunes target RPM (target shooting RPM when shooting, or target idle RPM when idle).
   - **Pulsed Transfer Shooting Strategy**: When shooting, the shooter flywheel ramps to 5200 RPM (adjustable) and the feeder runs continuously. After an initial `0.5s` delay (to let feeder prime), the transfer motor cycles ON for `0.5s` and OFF for `0.5s` repeatedly to prevent jam conditions.
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
  - `barLeft` — Servo (Hardware map name: `"barLeft"`, neutral: `0.5`, offset: `-0.25` = `0.25`)
  - `barRight` — Servo (Hardware map name: `"barRight"`, neutral: `0.5`, offset: `+0.25` = `0.75`)
- **Feeder & Transfer**:
  - `feeder` — Continuous Rotation Servo (Hardware map name: `"feeder"`, shoot power: `-1.0`, hold: `0.0`)
  - `transfer` — Motor (Hardware map name: `"transfer"`, intake power: `0.8`, outtake: `-0.6`, zero power behavior: `BRAKE`)
- **Shooter Flywheel**:
  - `shooter` — `DcMotorEx` with encoder feedback (Hardware map name: `"shooter"`, top speed: `6000.0` RPM, idle speed: `2500.0` RPM, shooting speed: `5200.0` RPM, zero power behavior: `FLOAT`)

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
  - `shooterRPM`: Prominent top parameter `%.0f / %.0f RPM (Pwr: %.2f)`.
  - `state`: Prominent top parameter `Mech: %s | AutoDrive: %s`.

### Loop Safety & Non-Blocking Rules
- **Strict Non-Blocking TeleOp**: `loop()` MUST be non-blocking. Never call `Thread.sleep()` or blocking `while` loops inside TeleOp.
- **Mandatory `follower.update()`**: `follower.update()` MUST be called once per loop iteration in all OpModes to maintain localizer tracking and motor control outputs.

### TeleOp Control Scheme & Button Mapping Standard
| Input / Gesture | Function / Behavior |
| :--- | :--- |
| **Left Stick Y/X** | Field-centric Translational Drive (Forward/Backward, Strafe Left/Right) |
| **Right Stick X** | Manual Heading Rotation (capped at 60% max speed; resets target heading on release after 350ms settling window) |
| **Left Trigger (`L2`)** | Analog Slow Driving Speed (scales max speed down to `slowModeMultiplier`) |
| **Left Trigger (`L2`) + D-Pad Up / Down** | Adjusts `slowModeMultiplier` in steps of `0.1` (clamped between `0.1` and `1.0`) |
| **Left Trigger (`L2`) + D-Pad Left / Right** | Adjusts target shooter RPM down/up in steps of `100` RPM (modifies shooting RPM when shooting, or idle RPM when idle) |
| **Y Button (without D-Pad)** | BallMechanism: Toggles flower release bar Up (`barUp()`, $0.5$) or Down (`barDown()`, $0.5 \pm 0.25$) |
| **X Button (without D-Pad)** | BallMechanism: **Toggles** `INTAKE` mode ON <-> OFF (`transfer` motor at $80\%$ / `0.8`, `feeder` holding $0.0$, shooter @ 2500 RPM) |
| **Left Bumper (`L1`) (without D-Pad)** | BallMechanism: State transition to `OUTTAKE` (`transfer` motor backwards $60\%$ / `-0.6`, `feeder` holding $0.0$, shooter @ 2500 RPM); turns intake OFF when released |
| **Right Bumper (`R1`) (without D-Pad)** | BallMechanism: State transition to `SHOOTING` (shooter @ 5200 RPM, `feeder` CRServo reverse $-1.0$, `transfer` motor pulsing $0.5\text{s}$ OFF / $0.5\text{s}$ ON); turns intake OFF when released |
| **Y + D-Pad Up / Down / Left / Right** | Saves current robot pose and heading to Slot 1 (Up), Slot 2 (Down), Slot 3 (Left), or Slot 4 (Right) |
| **A + D-Pad Up / Down / Left / Right** | Recalls and drives autonomously to user-saved pose Slot 1 (Up), Slot 2 (Down), Slot 3 (Left), or Slot 4 (Right) |
| **Right Bumper (`R1`) + D-Pad Up / Down / Left / Right** | Recalls pre-coded preset positions: Parking Blue (Up: 26.68, -49.84, 90°), Parking Red (Down: 99.27, 74.51, 270°), Shooting Area Left (Left: 17.21, 10.67, 360°), Shooting Area Right (Right: 113.19, 9.84, 180°) |
| **Left Bumper + Right Bumper + X** | Recalibrates robot position to origin `(0, 0, 0°)` |
| **B Button** | Aborts automated return-to-position drive |

---

## 4. Canonical Reference Examples

The following files represent the **golden reference implementations** for this repository:

1. **TeleOp Implementation**:
   - [SpectrixTeleOp.java](file:///C:/Users/lajos/Projects/QuickStart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixTeleOp.java) — Golden reference for Field-Centric TeleOp, Damped PD Heading Lock, Analog Slow Mode, Position Save/Return, and Subsystem integration.

2. **Mechanism Subsystem Implementation**:
   - [BallMechanism.java](file:///C:/Users/lajos/Projects/QuickStart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BallMechanism.java) — Golden reference for non-drivetrain mechanism encapsulation (`barLeft`, `barRight`, `feeder`, `transfer`, `shooter`).

3. **Standard Autonomous Implementation**:
   - [SpectrixAutoTest.java](file:///C:/Users/lajos/Projects/QuickStart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SpectrixAutoTest.java) — Golden reference for procedural `LinearOpMode` autonomous execution with Pedro Pathing.

4. **Hardware & Localizer Configuration**:
   - [Constants.java](file:///C:/Users/lajos/Projects/QuickStart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java) — Golden reference for Mecanum drivetrain, OctoQuad localizer, Foresight algorithm settings, and mechanism constants.
