package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Locale;

@TeleOp(name="Testing TeleOp (Freewheel)", group="Testing")
public class TestingTeleOp extends OpMode {
    private Follower follower;
    private BallMechanism ballMechanism;

    private double slowModeMultiplier = 0.5;

    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();

        // Initialize Ball Mechanism Subsystem
        ballMechanism = new BallMechanism(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Update localizer to read position
        follower.update();
        
        // Keep drivetrain floating and unpowered for testing
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        boolean dpadUpPressed = gamepad1.dpad_up && !previousDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !previousDpadDown;
        boolean dpadLeftPressed = gamepad1.dpad_left && !previousDpadLeft;
        boolean dpadRightPressed = gamepad1.dpad_right && !previousDpadRight;
        boolean dpadActive = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
        boolean xPressed = gamepad1.x && !previousX;
        boolean yPressed = gamepad1.y && !previousY;

        // Reset Pose Shortcut: Left Bumper + Right Bumper + X
        if (gamepad1.left_bumper && gamepad1.right_bumper && xPressed) {
            follower.setPose(Pose.zero());
            follower.hold(Pose.zero());
        }

        if (yPressed && !dpadActive) {
            // Toggle Bar Up / Down when Y is pressed without D-Pad
            ballMechanism.toggleBar();
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

        // Adjust Shooter Target RPM (D-Pad Left/Right)
        if (gamepad1.left_trigger > 0.5 && !gamepad1.y && !gamepad1.a && !gamepad1.right_bumper) {
            if (dpadRightPressed) {
                ballMechanism.adjustShooterRpm(100.0);
            }
            if (dpadLeftPressed) {
                ballMechanism.adjustShooterRpm(-100.0);
            }
        }

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
        telemetry.addData("state", String.format(Locale.US, "Mech: %s", ballMechanism.getCurrentState()));
        telemetry.addData("velocity", String.format(Locale.US, "VX: %.2f, VY: %.2f, W: %.2f°/s", velocity.vx, velocity.vy, Math.toDegrees(velocity.omega)));

        if (follower.drivetrain instanceof Mecanum) {
            Mecanum mecanum = (Mecanum) follower.drivetrain;
            telemetry.addData("wheels", String.format(Locale.US, "FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f", mecanum.wheelPowers[0], mecanum.wheelPowers[1], mecanum.wheelPowers[2], mecanum.wheelPowers[3]));
        }

        telemetry.addData("mechanism", String.format(Locale.US, "BarDown: %b | Stopper: %.2f | Transfer: %.2f | Feeder: %.2f", ballMechanism.isBarDown(), ballMechanism.getStopperPosition(), ballMechanism.getTransferPower(), ballMechanism.getFeederPower()));
        telemetry.update();
    }
}
