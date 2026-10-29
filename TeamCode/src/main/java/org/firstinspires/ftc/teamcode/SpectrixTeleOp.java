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

@TeleOp
public class SpectrixTeleOp extends OpMode {
    private Follower follower;
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

    // Heading Lock / Drift Correction PD Controller
    private double targetHeading = 0.0;
    private boolean isHeadingLocked = false;
    private double turnReleaseTime = 0.0;
    private static final double TURN_THRESHOLD = 0.05;
    private static final double SETTLING_DELAY_SEC = 0.25; // 250ms settling delay after stick release
    private static final double HEADING_P = 0.35;          // Smooth, gentle proportional gain
    private static final double HEADING_D = 0.02;          // Damping term to stop momentum
    private static final double HEADING_DEADBAND = Math.toRadians(1.5); // 1.5 degree deadband
    private static final double MAX_CORRECTION_POWER = 0.25;            // Cap correction power at 25% max

    // Target Arrival Tolerance for Automated Return Drive
    private static final double DISTANCE_TOLERANCE_INCHES = 2.0;
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(3.0); // 3 degrees
    private static final double LINEAR_VEL_TOLERANCE_IPS = 1.0;            // 1 inch/sec
    private static final double ANGULAR_VEL_TOLERANCE_RAD = Math.toRadians(5.0); // 5 deg/sec

    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPose == null ? Pose.zero() : startingPose);
        follower.update();
        savedPoseUp = Pose.zero();
        savedPoseDown = Pose.zero();
        savedPoseLeft = Pose.zero();
        savedPoseRight = Pose.zero();
        activeTargetPose = Pose.zero();
        activeTargetName = "";
        targetHeading = follower.pose().heading();
        isHeadingLocked = false;
        turnReleaseTime = 0.0;
    }

    @Override
    public void start() {
        // Mode will be handled in loop() via follower.manual(...) or follower.hold(...)
        targetHeading = follower.pose().heading();
        turnReleaseTime = getRuntime();
    }

    @Override
    public void loop() {
        // Call this once per loop
        follower.update();

        boolean dpadUpPressed = gamepad1.dpad_up && !previousDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !previousDpadDown;
        boolean dpadLeftPressed = gamepad1.dpad_left && !previousDpadLeft;
        boolean dpadRightPressed = gamepad1.dpad_right && !previousDpadRight;

        if (!automatedDrive) {
            // Calculate speed multiplier based on Left Trigger input
            double currentMultiplier = 1.0;
            if (gamepad1.left_trigger > 0.1) {
                currentMultiplier = 1.0 - (gamepad1.left_trigger * (1.0 - slowModeMultiplier));
            }

            double forward = -gamepad1.left_stick_y * currentMultiplier;
            double lateral = -gamepad1.left_stick_x * currentMultiplier;
            double manualTurn = -gamepad1.right_stick_x * currentMultiplier;

            double turnPower;
            if (Math.abs(gamepad1.right_stick_x) > TURN_THRESHOLD) {
                // Driver is manually turning -> Use manual input and record release time
                turnPower = manualTurn;
                turnReleaseTime = getRuntime();
                isHeadingLocked = false;
            } else {
                // Driver is driving straight/strafing without turning
                double timeSinceRelease = getRuntime() - turnReleaseTime;
                if (timeSinceRelease < SETTLING_DELAY_SEC) {
                    // Settling phase: allow robot to naturally coast to a stop without fighting momentum
                    turnPower = 0.0;
                    isHeadingLocked = false;
                } else {
                    // Robot has settled -> Capture current settled heading and apply gentle PD correction
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
            // Re-arm heading lock when automated drive finishes
            isHeadingLocked = false;
            turnReleaseTime = getRuntime();
        }

        // Reset Pose Shortcut: Left Bumper + Right Bumper + X
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.x && !previousX) {
            follower.setPose(Pose.zero());
            targetHeading = 0.0;
            isHeadingLocked = false;
            turnReleaseTime = getRuntime();
        }

        // Save Positions: Y + D-Pad Direction (Up = Pos 1, Down = Pos 2, Left = Pos 3, Right = Pos 4)
        if (gamepad1.y) {
            if (dpadUpPressed) savedPoseUp = follower.pose();
            if (dpadDownPressed) savedPoseDown = follower.pose();
            if (dpadLeftPressed) savedPoseLeft = follower.pose();
            if (dpadRightPressed) savedPoseRight = follower.pose();
        }

        // Recall Saved Positions: A + D-Pad Direction (Up = Pos 1, Down = Pos 2, Left = Pos 3, Right = Pos 4)
        if (gamepad1.a) {
            if (dpadUpPressed) startAutomatedDrive(savedPoseUp, "Saved Pos 1 (Up)");
            else if (dpadDownPressed) startAutomatedDrive(savedPoseDown, "Saved Pos 2 (Down)");
            else if (dpadLeftPressed) startAutomatedDrive(savedPoseLeft, "Saved Pos 3 (Left)");
            else if (dpadRightPressed) startAutomatedDrive(savedPoseRight, "Saved Pos 4 (Right)");
        }

        // Recall Pre-Coded Preset Positions: Right Bumper + D-Pad Direction
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            if (dpadUpPressed) startAutomatedDrive(PRESET_PARKING_BLUE, "Parking Blue");
            else if (dpadDownPressed) startAutomatedDrive(PRESET_PARKING_RED, "Parking Red");
            else if (dpadLeftPressed) startAutomatedDrive(PRESET_SHOOTING_AREA_LEFT, "Shooting Area Left");
            else if (dpadRightPressed) startAutomatedDrive(PRESET_SHOOTING_AREA_RIGHT, "Shooting Area Right");
        }

        // Check if automated return drive has reached the target position & settled
        if (automatedDrive) {
            double timeInAuto = getRuntime() - automatedStartTime;
            if (gamepad1.b && !previousB) {
                // Manual cancel
                follower.stop();
                automatedDrive = false;
                activeTargetName = "";
                targetHeading = follower.pose().heading();
                isHeadingLocked = false;
                turnReleaseTime = getRuntime();
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
                    isHeadingLocked = false;
                    turnReleaseTime = getRuntime();
                }
            }
        }

        // Adjust Slow Mode Strength: Left Trigger + D-Pad Up / Down (when not pressing Y, A, or RB)
        if (gamepad1.left_trigger > 0.5 && !gamepad1.y && !gamepad1.a && !gamepad1.right_bumper) {
            if (dpadUpPressed) {
                slowModeMultiplier = Math.min(1.0, slowModeMultiplier + 0.1);
            }
            if (dpadDownPressed) {
                slowModeMultiplier = Math.max(0.1, slowModeMultiplier - 0.1);
            }
        }

        previousB = gamepad1.b;
        previousX = gamepad1.x;
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;

        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();

        telemetry.addData("build", BuildConfig.BUILD_TIME);
        telemetry.addData("position", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("targetHeading", String.format(Locale.US, "%.2f°", Math.toDegrees(targetHeading)));
        telemetry.addData("velocity", String.format(Locale.US, "VX: %.2f, VY: %.2f, W: %.2f°/s", velocity.vx, velocity.vy, Math.toDegrees(velocity.omega)));
        telemetry.addData("savedUp (Pos 1)", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", savedPoseUp.x(), savedPoseUp.y(), Math.toDegrees(savedPoseUp.heading())));
        telemetry.addData("savedDown (Pos 2)", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", savedPoseDown.x(), savedPoseDown.y(), Math.toDegrees(savedPoseDown.heading())));
        telemetry.addData("savedLeft (Pos 3)", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", savedPoseLeft.x(), savedPoseLeft.y(), Math.toDegrees(savedPoseLeft.heading())));
        telemetry.addData("savedRight (Pos 4)", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", savedPoseRight.x(), savedPoseRight.y(), Math.toDegrees(savedPoseRight.heading())));
        telemetry.addData("slowLimit", String.format(Locale.US, "%.1f", slowModeMultiplier));
        telemetry.addData("automatedDrive", automatedDrive ? activeTargetName : "FALSE");
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
