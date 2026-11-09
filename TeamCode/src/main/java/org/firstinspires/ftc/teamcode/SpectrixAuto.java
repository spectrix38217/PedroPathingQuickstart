package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.line;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * SpectrixAuto - Main Autonomous OpMode using Pedro Pathing and BallMechanism subsystem.
 *
 * <p>Execution Flow:
 * 1. Initialize Follower and BallMechanism.
 * 2. Set starting pose (0, 0, 0°).
 * 3. Execute autonomous paths and mechanism actions.
 * 4. Save final robot pose to OpModeStorage & SpectrixTeleOp.startingPose upon completion
 *    for seamless transition into TeleOp without localizer drift.
 */
@Autonomous(name = "SpectrixAuto", group = "Autonomous")
public class SpectrixAuto extends LinearOpMode {

    private Follower follower;
    private BallMechanism ballMechanism;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Step 1: Initialize Pedro Pathing Follower and BallMechanism Subsystem.
         */
        follower = Constants.create(hardwareMap);
        ballMechanism = new BallMechanism(hardwareMap);

        Pose startPose = new Pose(0, 0, 0); // Starting position (X: 0 in, Y: 0 in, Heading: 0°)
        follower.setPose(startPose);

        /*
         * Step 2: Define Autonomous Paths using Pedro Pathing API.
         */
        // Path 1: Drive forward 24 inches keeping constant 0° heading
        Pose pose1 = new Pose(24, 0, 0);
        Path path1 = line(startPose, pose1).constant(0);

        // Path 2: Strafe left 12 inches while turning 90° counter-clockwise (PI/2 radians)
        double ccw90Deg = Math.toRadians(90);
        Pose pose2 = new Pose(24, 12, ccw90Deg);
        Path path2 = line(pose1, pose2).linear(0, ccw90Deg);

        // Path 3: Return to origin (0, 0, 0°)
        Path path3 = line(pose2, startPose).linear(ccw90Deg, 0);

        telemetry.addData("Status", "SpectrixAuto Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        /*
         * Step 3: Wait for Driver to press PLAY on Driver Station.
         */
        waitForStart();

        if (isStopRequested()) return;

        /*
         * Step 4: Execute Autonomous Sequence.
         */

        // --- Action 1: Drive forward 24 inches ---
        followPath(path1, "1/3: Driving Forward 24 Inches");

        // --- Pause 1 second ---
        waitSeconds(1.0, "Waiting 1 Second");

        // --- Action 2: Strafe left 12 inches & turn 90° ---
        followPath(path2, "2/3: Strafing Left & Turning 90°");

        // --- Action 3: Return to origin (0,0) @ 0° ---
        followPath(path3, "3/3: Returning to Origin (0,0)");

        /*
         * Step 5: Save Final Robot Pose for TeleOp Transition (CRITICAL).
         * Storing follower.pose() into OpModeStorage and SpectrixTeleOp.startingPose
         * allows TeleOp to start with the exact position and heading where Autonomous ended!
         */
        follower.stop();
        ballMechanism.idle();

        Pose finalPose = follower.pose();
        OpModeStorage.autonomousEndPose = finalPose;
        SpectrixTeleOp.startingPose = finalPose;

        while (opModeIsActive()) {
            follower.update();
            ballMechanism.update();

            telemetry.addData("Status", "Autonomous Complete!");
            telemetry.addData("Final Saved Pose", String.format(Locale.US, "X: %.2f in, Y: %.2f in, H: %.2f°", finalPose.x(), finalPose.y(), Math.toDegrees(finalPose.heading())));
            telemetry.update();
        }
    }

    /**
     * Helper function to follow a path to completion while continuously updating
     * the follower, localizer, and ball mechanism subsystem.
     */
    private void followPath(Path path, String stepName) {
        follower.follow(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            ballMechanism.update();

            Pose pose = follower.pose();
            telemetry.addData("Step", stepName);
            telemetry.addData("Position", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            telemetry.addData("Velocity", follower.velocity());
            telemetry.update();
        }
    }

    /**
     * Helper function to pause execution for a given duration in seconds.
     */
    private void waitSeconds(double seconds, String stepName) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
            ballMechanism.update();

            telemetry.addData("Step", stepName);
            telemetry.addData("Wait Progress", String.format(Locale.US, "%.1f / %.1f sec", timer.seconds(), seconds));
            telemetry.addData("Position", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
            telemetry.update();
        }
    }
}
