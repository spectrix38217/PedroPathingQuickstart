package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

@TeleOp(name="0 - Main TeleOp", group="TeleOp")
public class SpectrixTeleOp extends OpMode {
    private Follower follower;
    private BallMechanism ballMechanism;

    public static Pose startingPose; // See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private double automatedStartTime = 0.0;
    private double slowModeMultiplier = 0.5;

    // 4 Distinct Saved Pose Slots (Up = Slot 1, Down = Slot 2, Left = Slot 3, Right = Slot 4)
    private Pose savedPoseUp = Pose.zero();
    private Pose savedPoseDown = Pose.zero();
    private Pose savedPoseLeft = Pose.zero();
    private Pose savedPoseRight = Pose.zero();

    // 4 Pre-Coded Preset Positions (Recalled via Right Bumper + D-Pad Arrow)
    private static final Pose PRESET_PARKING_BLUE = new Pose(26.68, -49.84, Math.toRadians(90.0));
    private static final Pose PRESET_PARKING_RED = new Pose(99.27, 74.51, Math.toRadians(270.0));
    private static final Pose PRESET_SHOOTING_AREA_LEFT = new Pose(17.21, 10.67, Math.toRadians(360.0));
    private static final Pose PRESET_SHOOTING_AREA_RIGHT = new Pose(113.19, 9.84, Math.toRadians(180.0));

    private Pose activeTargetPose = Pose.zero();
    private String activeTargetName = "";

    // Absolute Pointing Rotation & Snap-Turn Gains (Smooth, non-oscillating rotation)
    private static final double ABSOLUTE_HEADING_P = 0.45;       // Smooth proportional gain for angle pointing
    private static final double ABSOLUTE_HEADING_D = 0.08;       // High derivative damping to eliminate overshoot
    private static final double MAX_ABSOLUTE_TURN_POWER = 0.50;  // Cap turning power at 50% for stable rotation

    // Heading & Position Lock Controller (Defense Resistance)
    private Pose targetPositionPose = Pose.zero();
    private double targetHeading = 0.0;
    private boolean isPositionLocked = false;
    private boolean isHeadingLocked = false;
    private boolean isSnapTurning = false;
    private double driveReleaseTime = 0.0;
    private static final double STICK_THRESHOLD = 0.05;
    private static final double POINTING_THRESHOLD = 0.20;       // Right stick activation threshold
    private static final double SETTLING_DELAY_SEC = 0.35;       // 350ms settling delay for heavier robot
    private static final double HEADING_P = 0.30;                // Smooth proportional gain for heading hold
    private static final double HEADING_D = 0.04;                // Damping for higher rotational inertia
    private static final double HEADING_DEADBAND = Math.toRadians(1.5); // 1.5 degree deadband
    private static final double MAX_CORRECTION_POWER = 0.20;            // Cap correction power at 20% max

    // Target Arrival Tolerance for Automated Return Drive
    private static final double DISTANCE_TOLERANCE_INCHES = 2.0;
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(3.0); // 3 degrees
    private static final double LINEAR_VEL_TOLERANCE_IPS = 1.0;            // 1 inch/sec
    private static final double ANGULAR_VEL_TOLERANCE_RAD = Math.toRadians(5.0); // 5 deg/sec

    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose((startingPose == null) ? Pose.zero() : startingPose);
        follower.update();

        // Initialize Ball Mechanism Subsystem
        ballMechanism = new BallMechanism(hardwareMap);

        savedPoseUp = Pose.zero();
        savedPoseDown = Pose.zero();
        savedPoseLeft = Pose.zero();
        savedPoseRight = Pose.zero();
        activeTargetPose = Pose.zero();
        activeTargetName = "";
        targetPositionPose = follower.pose();
        targetHeading = follower.pose().heading();
        isPositionLocked = false;
        isHeadingLocked = false;
        isSnapTurning = false;
        driveReleaseTime = 0.0;
    }

    @Override
    public void start() {
        // Mode will be handled in loop() via follower.manual(...) or follower.hold(...)
        targetPositionPose = follower.pose();
        targetHeading = follower.pose().heading();
        driveReleaseTime = getRuntime();
    }

    @Override
    public void loop() {
        // Call this once per loop
        follower.update();

        boolean dpadUpPressed = gamepad1.dpad_up && !previousDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !previousDpadDown;
        boolean dpadLeftPressed = gamepad1.dpad_left && !previousDpadLeft;
        boolean dpadRightPressed = gamepad1.dpad_right && !previousDpadRight;
        boolean dpadActive = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
        boolean xPressed = gamepad1.x && !previousX;
        boolean yPressed = gamepad1.y && !previousY;

        double forwardInput = -gamepad1.left_stick_y;
        double lateralInput = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x; // Flipped X component so Left/Right match field orientation
        double ry = -gamepad1.right_stick_y;  // Inverted Y component so UP is positive

        double rightStickMagnitude = Math.hypot(rx, ry);
        boolean isDriving = Math.hypot(forwardInput, lateralInput) > STICK_THRESHOLD;
        boolean isPointingHeading = rightStickMagnitude >= POINTING_THRESHOLD;

        // If driver touches the sticks, cancel any ongoing automatic snap turn
        if (isPointingHeading || isDriving) {
            isSnapTurning = false;
        }

        // Check for D-Pad Snap-Turn triggers (Pressed alone without modifiers)
        boolean dpadAloneActive = !gamepad1.y && !gamepad1.a && !gamepad1.right_bumper && gamepad1.left_trigger <= 0.5;
        if (dpadAloneActive) {
            if (dpadUpPressed) {
                targetHeading = 0.0; // 0 degrees
                isSnapTurning = true;
            } else if (dpadLeftPressed) {
                targetHeading = Math.toRadians(90.0); // 90 degrees (Left)
                isSnapTurning = true;
            } else if (dpadDownPressed) {
                targetHeading = Math.toRadians(180.0); // 180 degrees (Down)
                isSnapTurning = true;
            } else if (dpadRightPressed) {
                targetHeading = Math.toRadians(270.0); // 270 degrees (Right)
                isSnapTurning = true;
            }
        }

        if (!automatedDrive) {
            // Calculate speed multiplier based on Left Trigger input
            double currentMultiplier = (gamepad1.left_trigger > 0.1) ? (1.0 - (gamepad1.left_trigger * (1.0 - slowModeMultiplier))) : 1.0;

            double forward = forwardInput * currentMultiplier;
            double lateral = lateralInput * currentMultiplier;

            if (isDriving || isPointingHeading || isSnapTurning) {
                // Driver is actively steering, pointing right stick, or executing a snap turn -> Manual Drive
                driveReleaseTime = getRuntime();
                isPositionLocked = false;

                double turnPower;
                if (isSnapTurning) {
                    // Automated D-Pad Snap Turn to targetHeading
                    double headingError = Angle.error(follower.pose().heading(), targetHeading);
                    if (Math.abs(headingError) > HEADING_DEADBAND) {
                        double angularVelocity = follower.velocity().omega;
                        double turnPD = (headingError * ABSOLUTE_HEADING_P) - (angularVelocity * ABSOLUTE_HEADING_D);
                        turnPower = Math.min(MAX_ABSOLUTE_TURN_POWER, Math.max(-MAX_ABSOLUTE_TURN_POWER, turnPD));
                    } else {
                        turnPower = 0.0;
                        isSnapTurning = false; // Reached target heading!
                    }
                } else if (isPointingHeading) {
                    // Right stick pointing steering
                    targetHeading = Math.atan2(rx, ry);

                    double headingError = Angle.error(follower.pose().heading(), targetHeading);
                    if (Math.abs(headingError) > HEADING_DEADBAND) {
                        double angularVelocity = follower.velocity().omega;
                        double turnPD = (headingError * ABSOLUTE_HEADING_P) - (angularVelocity * ABSOLUTE_HEADING_D);
                        turnPower = Math.min(MAX_ABSOLUTE_TURN_POWER, Math.max(-MAX_ABSOLUTE_TURN_POWER, turnPD));
                    } else {
                        turnPower = 0.0;
                    }
                } else {
                    // Driver is driving straight/strafing without turning -> Damped PD Heading Lock
                    double timeSinceRelease = getRuntime() - driveReleaseTime;
                    if (timeSinceRelease < SETTLING_DELAY_SEC) {
                        turnPower = 0.0;
                        isHeadingLocked = false;
                    } else {
                        if (!isHeadingLocked) {
                            targetHeading = follower.pose().heading();
                            isHeadingLocked = true;
                        }

                        double headingError = Angle.error(follower.pose().heading(), targetHeading);
                        if (Math.abs(headingError) > HEADING_DEADBAND) {
                            double angularVelocity = follower.velocity().omega;
                            double turnPD = (headingError * HEADING_P) - (angularVelocity * HEADING_D);
                            turnPower = Math.min(MAX_CORRECTION_POWER, Math.max(-MAX_CORRECTION_POWER, turnPD));
                        } else {
                            turnPower = 0.0;
                        }
                    }
                }

                DrivePowers powers = ManualDrive.fieldCentric(
                        forward,
                        lateral,
                        turnPower,
                        follower.pose().heading()
                );

                follower.manual(powers);
            } else {
                // Driver is NOT touching sticks -> Position & Heading Lock (Resists pushing)
                double timeSinceRelease = getRuntime() - driveReleaseTime;
                if (timeSinceRelease < SETTLING_DELAY_SEC) {
                    // Settling phase: allow heavier robot to naturally coast to a stop without fighting momentum
                    follower.manual(DrivePowers.zero());
                    isPositionLocked = false;
                    isHeadingLocked = false;
                } else {
                    // Robot has settled -> Lock position (X, Y) and heading (H) using follower.hold()
                    if (!isPositionLocked) {
                        targetPositionPose = follower.pose();
                        targetHeading = targetPositionPose.heading();
                        isPositionLocked = true;
                        isHeadingLocked = true;
                    }

                    follower.hold(targetPositionPose);
                }
            }
        } else {
            // Re-arm heading/position lock when automated drive finishes
            isPositionLocked = false;
            isHeadingLocked = false;
            isSnapTurning = false;
            driveReleaseTime = getRuntime();
        }

        // Reset Pose Shortcut: Left Bumper + Right Bumper + X
        if (gamepad1.left_bumper && gamepad1.right_bumper && xPressed) {
            follower.setPose(Pose.zero());
            targetPositionPose = Pose.zero();
            targetHeading = 0.0;
            isPositionLocked = false;
            isHeadingLocked = false;
            isSnapTurning = false;
            driveReleaseTime = getRuntime();
        }

        // Save Positions: Y + D-Pad Direction (Up = Pos 1, Down = Pos 2, Left = Pos 3, Right = Pos 4)
        if (gamepad1.y && dpadActive) {
            if (dpadUpPressed) savedPoseUp = follower.pose();
            if (dpadDownPressed) savedPoseDown = follower.pose();
            if (dpadLeftPressed) savedPoseLeft = follower.pose();
            if (dpadRightPressed) savedPoseRight = follower.pose();
        } else if (yPressed && !dpadActive) {
            // Toggle Bar Up / Down when Y is pressed without D-Pad
            ballMechanism.toggleBar();
        }

        // Recall Saved Positions: A + D-Pad Direction (Up = Pos 1, Down = Pos 2, Left = Pos 3, Right = Pos 4)
        if (gamepad1.a) {
            if (dpadUpPressed) startAutomatedDrive(savedPoseUp, "Saved Pos 1 (Up)");
            else if (dpadDownPressed) startAutomatedDrive(savedPoseDown, "Saved Pos 2 (Down)");
            else if (dpadLeftPressed) startAutomatedDrive(savedPoseLeft, "Saved Pos 3 (Left)");
            else if (dpadRightPressed) startAutomatedDrive(savedPoseRight, "Saved Pos 4 (Right)");
        }

        // Recall Pre-Coded Preset Positions: Right Bumper + D-Pad Direction
        if (gamepad1.right_bumper && !gamepad1.left_bumper && dpadActive) {
            if (dpadUpPressed) startAutomatedDrive(PRESET_PARKING_BLUE, "Parking Blue");
            else if (dpadDownPressed) startAutomatedDrive(PRESET_PARKING_RED, "Parking Red");
            else if (dpadLeftPressed) startAutomatedDrive(PRESET_SHOOTING_AREA_LEFT, "Shooting Area Left");
            else if (dpadRightPressed) startAutomatedDrive(PRESET_SHOOTING_AREA_RIGHT, "Shooting Area Right");
        }

        // Ball Mechanism Controls (Intake Toggle Normal/Reverse, Outtake, Shooting)
        if (gamepad1.right_bumper && !dpadActive && !gamepad1.left_bumper) {
            // Right Bumper alone: Shoot (Shooter at 5200 RPM) - turns intake off when released
            ballMechanism.startShooting();
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.x) {
            // Left Bumper alone: Outtake (Transfer motor backwards -0.6) - turns intake off when released
            ballMechanism.startOuttake();
        } else if (xPressed && !dpadActive && !gamepad1.right_bumper) {
            // X button: Pressing X alone toggles Normal Intake; pressing LB + X toggles Reverse Intake
            if (gamepad1.left_bumper) {
                ballMechanism.toggleIntakeReverse();
            } else {
                ballMechanism.toggleIntakeNormal();
            }
        } else if (ballMechanism.getCurrentState() != BallMechanism.State.INTAKE && ballMechanism.getCurrentState() != BallMechanism.State.INTAKE_REVERSE) {
            // Idle if not intaking (normal or reverse), shooting, or outtaking
            ballMechanism.idle();
        }

        // Update Mechanism Subsystem State Machine & Shooter Controller
        ballMechanism.update();

        // Check if automated return drive has reached the target position & settled
        if (automatedDrive) {
            double timeInAuto = getRuntime() - automatedStartTime;
            if (gamepad1.b && !previousB) {
                // Manual cancel
                follower.stop();
                automatedDrive = false;
                activeTargetName = "";
                targetHeading = follower.pose().heading();
                isPositionLocked = false;
                isHeadingLocked = false;
                isSnapTurning = false;
                driveReleaseTime = getRuntime();
            } else if (timeInAuto > 0.3) { // Minimum 300ms drive window before checking target arrival
                double distanceError = follower.pose().distance(activeTargetPose);
                double headingError = Math.abs(Angle.error(follower.pose().heading(), activeTargetPose.heading()));
                double linearSpeed = follower.velocity().toVector2D().magnitude();
                double angularSpeed = Math.abs(follower.velocity().omega);

                boolean isAtTarget = distanceError < DISTANCE_TOLERANCE_INCHES
                        && headingError < HEADING_TOLERANCE_RAD
                        && linearSpeed < LINEAR_VEL_TOLERANCE_IPS
                        && angularSpeed < ANGULAR_VEL_TOLERANCE_RAD;

                if (isAtTarget) {
                    follower.stop();
                    automatedDrive = false;
                    activeTargetName = "";
                    targetHeading = follower.pose().heading();
                    isPositionLocked = false;
                    isHeadingLocked = false;
                    isSnapTurning = false;
                    driveReleaseTime = getRuntime();
                }
            }
        }

        // Adjust Slow Mode Strength (D-Pad Up/Down) & Shooter Target RPM (D-Pad Left/Right)
        if (gamepad1.left_trigger > 0.5 && !gamepad1.y && !gamepad1.a && !gamepad1.right_bumper) {
            if (dpadUpPressed) {
                slowModeMultiplier = Math.min(1.0, slowModeMultiplier + 0.1);
            }
            if (dpadDownPressed) {
                slowModeMultiplier = Math.max(0.1, slowModeMultiplier - 0.1);
            }
            if (dpadRightPressed) {
                ballMechanism.adjustShooterRpm(100.0);
            }
            if (dpadLeftPressed) {
                ballMechanism.adjustShooterRpm(-100.0);
            }
        }

        previousB = gamepad1.b;
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;

        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();

        telemetry.addData("build", BuildConfig.BUILD_TIME);
        telemetry.addData("position", String.format(Locale.US, "X: %.2f in, Y: %.2f in, H: %.2f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("shooterRPM", String.format(Locale.US, "%.0f / %.0f RPM [ShootTarget: %.0f RPM] (Pwr: %.2f)", ballMechanism.getCurrentShooterRpm(), ballMechanism.getTargetShootRpm(), ballMechanism.getTargetShootRpm(), ballMechanism.getShooterPower()));
        telemetry.addData("state", String.format(Locale.US, "Mech: %s | AutoDrive: %s", ballMechanism.getCurrentState(), automatedDrive ? activeTargetName : "OFF"));
        telemetry.addData("posLock", isPositionLocked ? String.format(Locale.US, "X: %.2f, Y: %.2f", targetPositionPose.x(), targetPositionPose.y()) : "OFF");
        telemetry.addData("targetHeading", String.format(Locale.US, "%.2f°", Math.toDegrees(targetHeading)));
        telemetry.addData("velocity", String.format(Locale.US, "VX: %.2f, VY: %.2f, W: %.2f°/s", velocity.vx, velocity.vy, Math.toDegrees(velocity.omega)));
        telemetry.addData("savedUp (Pos 1)", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", savedPoseUp.x(), savedPoseUp.y(), Math.toDegrees(savedPoseUp.heading())));
        telemetry.addData("savedDown (Pos 2)", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", savedPoseDown.x(), savedPoseDown.y(), Math.toDegrees(savedPoseDown.heading())));
        telemetry.addData("savedLeft (Pos 3)", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", savedPoseLeft.x(), savedPoseLeft.y(), Math.toDegrees(savedPoseLeft.heading())));
        telemetry.addData("savedRight (Pos 4)", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", savedPoseRight.x(), savedPoseRight.y(), Math.toDegrees(savedPoseRight.heading())));
        telemetry.addData("mechanism", String.format(Locale.US, "BarDown: %b | Transfer: %.2f | Feeder: %.2f", ballMechanism.isBarDown(), ballMechanism.getTransferPower(), ballMechanism.getFeederPower()));
        telemetry.addData("slowLimit", String.format(Locale.US, "%.1f", slowModeMultiplier));
        telemetry.update();
    }

    private void startAutomatedDrive(Pose target, String name) {
        activeTargetPose = target;
        activeTargetName = name;
        follower.hold(target);
        automatedDrive = true;
        automatedStartTime = getRuntime();
    }
}
