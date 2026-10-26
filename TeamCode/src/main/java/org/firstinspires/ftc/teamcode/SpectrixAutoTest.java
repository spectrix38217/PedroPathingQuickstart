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
 * SpectrixAutoTest - Example Autonomous OpMode using Pedro Pathing.
 *
 * <p>Linear Execution Flow:
 * 1. Drive forward 1 foot (12 inches).
 * 2. Wait for 1 second.
 * 3. Strafe left 1 foot (12 inches) and turn 90 degrees clockwise (-90 degrees / 270 degrees).
 * 4. Wait for 2 seconds.
 * 5. Return to starting position (0,0) and reset heading to 0 degrees.
 * 6. Save final pose to OpModeStorage & SpectrixTeleOp for seamless transition to TeleOp!
 */
@Autonomous(name = "SpectrixAutoTest", group = "Autonomous")
public class SpectrixAutoTest extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Step 1: Initialize the Pedro Pathing follower from Constants.
         * We set the starting pose to origin (0, 0) facing 0 degrees.
         */
        follower = Constants.create(hardwareMap);
        Pose startPose = Pose.zero(); // (0, 0, 0 degrees)
        follower.setPose(startPose);

        /*
         * Step 2: Define Autonomous Paths using Pedro Pathing API.
         *
         * Coordinate System Notes:
         * - X is Forward/Backward (+X is Forward).
         * - Y is Left/Right (+Y is Left).
         * - Heading is in Radians Counter-Clockwise (+90° is Left, -90° / 270° is Clockwise).
         */

        // Path 1: Drive forward 1 foot (12 inches), keeping constant 0° heading.
        Pose pose1 = new Pose(12, 0, 0);
        Path path1 = line(startPose, pose1).constant(0);

        // Path 2: Strafe left 1 foot (12 inches) to (12, 12) while turning 90° clockwise (-90° / -PI/2 radians).
        double clockwise90Deg = Math.toRadians(-90); // -90 degrees in radians
        Pose pose2 = new Pose(12, 12, clockwise90Deg);
        Path path2 = line(pose1, pose2).linear(0, clockwise90Deg);

        // Path 3: Return to starting position (0, 0) and reset heading to 0°.
        Path path3 = line(pose2, startPose).linear(clockwise90Deg, 0);

        // Telemetry during initialization
        telemetry.addData("Status", "Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        /*
         * Step 3: Wait for Driver to press PLAY on Driver Station.
         */
        waitForStart();

        if (isStopRequested()) return;

        /*
         * Step 4: Execute Autonomous Command Sequence linearly.
         */

        // --- Action 1: Drive forward 1 foot ---
        followPath(path1, "1/5: Driving Forward 1 Foot");

        // --- Action 2: Wait 1 second ---
        waitSeconds(1.0, "2/5: Waiting 1 Second");

        // --- Action 3: Strafe left 1 foot & turn 90° clockwise ---
        followPath(path2, "3/5: Strafing Left 1 Foot & Turning 90° Clockwise");

        // --- Action 4: Wait 2 seconds ---
        waitSeconds(2.0, "4/5: Waiting 2 Seconds");

        // --- Action 5: Return to starting position (0,0) and reset heading to 0° ---
        followPath(path3, "5/5: Returning to Origin (0,0) @ 0° Heading");

        /*
         * Step 5: Stop motors and save Final Robot Pose for TeleOp Transition.
         * By storing follower.pose() into OpModeStorage and SpectrixTeleOp.startingPose,
         * TeleOp will automatically pick up where Autonomous finished without localizer drift!
         */
        follower.stop(); // Stop motor power at the end of autonomous!
        Pose finalPose = follower.pose();
        OpModeStorage.autonomousEndPose = finalPose;
        SpectrixTeleOp.startingPose = finalPose;

        while (opModeIsActive()) {
            follower.update(); // Keep localizer updating until driver hits STOP on DS
            telemetry.addData("Status", "Autonomous Complete!");
            telemetry.addData("Final Saved Pose", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", finalPose.x(), finalPose.y(), Math.toDegrees(finalPose.heading())));
            telemetry.update();
        }
    }

    /**
     * Helper function to follow a path to completion while continuously updating
     * Pedro Pathing localizer/follower and logging telemetry.
     *
     * @param path The Pedro Pathing Path to follow
     * @param stepName Description of current step for telemetry display
     */
    private void followPath(Path path, String stepName) {
        follower.follow(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update(); // MUST be called every loop to update localizer & motors!

            Pose pose = follower.pose();
            telemetry.addData("Step", stepName);
            telemetry.addData("Position", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            telemetry.addData("Velocity", follower.velocity());
            telemetry.update();
        }
    }

    /**
     * Helper function to pause execution for a given duration in seconds.
     * Keeps follower.update() running during pauses so odometry tracking remains active.
     *
     * @param seconds Time to wait in seconds
     * @param stepName Description of current step for telemetry display
     */
    private void waitSeconds(double seconds, String stepName) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update(); // Keeps localizer active and motors holding position during pause!

            telemetry.addData("Step", stepName);
            telemetry.addData("Wait Progress", String.format(Locale.US, "%.1f / %.1f sec", timer.seconds(), seconds));
            telemetry.addData("Position", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
            telemetry.update();
        }
    }
}
