package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

@TeleOp
public class SpectrixTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private double triggerSlowModeMultiplier = 0.35;
    private Pose savedPose;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(54, 94))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                .build();
        //Lazy curve generation might look a little different to how we usually make paths,
        //This is because we are using a Lambda expression instead of calling follower.pathbuilder()
        //To use this though, we declare this as a Supplier<PathChain>. And we to call the path chain we instead do pathChain.get()
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        if (!automatedDrive) {
            // Check for L2 (left_trigger) slow mode
            double currentMultiplier = 1.0;
            if (gamepad1.left_trigger > 0.1) {
                currentMultiplier = 1.0 - (gamepad1.left_trigger * (1.0 - triggerSlowModeMultiplier));
            } else if (slowMode) {
                currentMultiplier = slowModeMultiplier;
            }

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * currentMultiplier,
                    -gamepad1.left_stick_x * currentMultiplier,
                    -gamepad1.right_stick_x * currentMultiplier,
                    false // Robot Centric
            );
        }
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        // Reset Pose
        if (gamepad1.leftBumperWasPressed()) {
            follower.setPose(new Pose(0, 0, 0));
        }

        // Save/Return to position
        if (gamepad1.right_trigger > 0.5) {
            if (gamepad1.yWasPressed()) { // Triangle
                savedPose = follower.getPose();
            }
            if (gamepad1.xWasPressed() && savedPose != null) { // Square
                follower.followPath(follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower.getPose(), savedPose)))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), savedPose.getHeading())
                        .build());
                automatedDrive = true;
            }
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed() && !(gamepad1.right_trigger > 0.5)) {
            slowModeMultiplier += 0.25;
        }
        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}