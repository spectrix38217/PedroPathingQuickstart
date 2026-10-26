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
    private Pose savedPose = Pose.zero();

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

    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPose == null ? Pose.zero() : startingPose);
        follower.update();
        savedPose = Pose.zero();
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

        // Trigger Combos (Right Trigger > 0.5)
        if (gamepad1.right_trigger > 0.5) {
            // Right Trigger + Y: Save current position and heading
            if (gamepad1.y && !previousY) {
                savedPose = follower.pose();
            }
            // Right Trigger + A: Ease onto saved position and heading accurately
            if (gamepad1.a && !previousA) {
                follower.hold(savedPose);
                automatedDrive = true;
                automatedStartTime = getRuntime();
            }
        }

        // Check if automated return drive has reached the target position & settled
        if (automatedDrive) {
            double timeInAuto = getRuntime() - automatedStartTime;
            if (gamepad1.b && !previousB) {
                // Manual cancel
                follower.stop();
                automatedDrive = false;
                targetHeading = follower.pose().heading();
                isHeadingLocked = false;
                turnReleaseTime = getRuntime();
            } else if (timeInAuto > 0.3) { // Minimum 300ms drive window before checking target arrival
                double distanceError = follower.pose().distance(savedPose);
                double headingError = Math.abs(Angle.error(follower.pose().heading(), savedPose.heading()));
                double linearSpeed = follower.velocity().toVector2D().magnitude();
                double angularSpeed = Math.abs(follower.velocity().omega);

                boolean isAtTarget = distanceError < DISTANCE_TOLERANCE_INCHES
                        && headingError < HEADING_TOLERANCE_RAD
                        && linearSpeed < LINEAR_VEL_TOLERANCE_IPS
                        && angularSpeed < ANGULAR_VEL_TOLERANCE_RAD;

                if (isAtTarget) {
                    follower.stop();
                    automatedDrive = false;
                    targetHeading = follower.pose().heading();
                    isHeadingLocked = false;
                    turnReleaseTime = getRuntime();
                }
            }
        }

        // Adjust Slow Mode Strength: Left Trigger + D-Pad Up / Down
        if (gamepad1.left_trigger > 0.5) {
            if (gamepad1.dpad_up && !previousDpadUp) {
                slowModeMultiplier = Math.min(1.0, slowModeMultiplier + 0.1);
            }
            if (gamepad1.dpad_down && !previousDpadDown) {
                slowModeMultiplier = Math.max(0.1, slowModeMultiplier - 0.1);
            }
        }

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;

        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();

        telemetry.addData("build", BuildConfig.BUILD_TIME);
        telemetry.addData("position", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("targetHeading", String.format(Locale.US, "%.2f°", Math.toDegrees(targetHeading)));
        telemetry.addData("velocity", String.format(Locale.US, "VX: %.2f, VY: %.2f, W: %.2f°/s", velocity.vx, velocity.vy, Math.toDegrees(velocity.omega)));
        telemetry.addData("savedPosition", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", savedPose.x(), savedPose.y(), Math.toDegrees(savedPose.heading())));
        telemetry.addData("slowLimit", String.format(Locale.US, "%.1f", slowModeMultiplier));
        telemetry.addData("automatedDrive", automatedDrive);
        telemetry.update();
    }
}
