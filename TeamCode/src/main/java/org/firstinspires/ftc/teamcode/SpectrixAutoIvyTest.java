package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.line;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Locale;

/**
 * SpectrixAutoIvyTest - Example Autonomous OpMode using the Ivy Command-Based Framework.
 *
 * <p>Ivy is Pedro Pathing's command-based control framework for composing autonomous actions.
 *
 * <p>Linear Execution Flow via Ivy Sequential Command:
 * 1. Drive forward 1 foot (12 inches).
 * 2. Wait for 1 second (1000 ms).
 * 3. Strafe left 1 foot (12 inches) and turn 90 degrees clockwise (-90° / 270°).
 * 4. Wait for 2 seconds (2000 ms).
 * 5. Return to starting position (0,0) and reset heading to 0 degrees.
 * 6. Save final pose to OpModeStorage & SpectrixTeleOp for seamless transition to TeleOp!
 */
@Autonomous(name = "SpectrixAutoIvyTest", group = "Autonomous")
public class SpectrixAutoIvyTest extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        /*
         * CRITICAL IVY REQUIREMENT:
         * Reset the Ivy Scheduler before every OpMode run to clear lingering commands
         * from previous runs.
         */
        Scheduler.reset();

        /*
         * Step 1: Initialize Pedro Pathing Follower and set starting pose at origin (0, 0, 0°).
         */
        follower = Constants.create(hardwareMap);
        Pose startPose = Pose.zero(); // (0, 0, 0 degrees)
        follower.setPose(startPose);

        /*
         * Step 2: Define Pedro Pathing Paths.
         */
        // Path 1: Drive forward 1 foot (12 inches), keeping constant 0° heading.
        Pose pose1 = new Pose(12, 0, 0);
        Path path1 = line(startPose, pose1).constant(0);

        // Path 2: Strafe left 1 foot (12 inches) to (12, 12) while turning 90° clockwise (-90° / -PI/2 radians).
        double clockwise90Deg = Math.toRadians(-90);
        Pose pose2 = new Pose(12, 12, clockwise90Deg);
        Path path2 = line(pose1, pose2).linear(0, clockwise90Deg);

        // Path 3: Return to starting position (0, 0) and reset heading to 0°.
        Path path3 = line(pose2, startPose).linear(clockwise90Deg, 0);

        /*
         * Step 3: Build Ivy Command Sequence using sequential composition.
         */
        Command autoSequence = sequential(
                // Action 1: Drive forward 1 foot
                followPath(follower, path1),

                // Action 2: Wait 1 second (1000 ms) while updating localizer
                waitMsWithUpdate(follower, 1000),

                // Action 3: Strafe left 1 foot & turn 90° clockwise
                followPath(follower, path2),

                // Action 4: Wait 2 seconds (2000 ms) while updating localizer
                waitMsWithUpdate(follower, 2000),

                // Action 5: Return to starting position (0,0) and reset heading to 0°
                followPath(follower, path3),

                // Action 6: Stop motors & save final robot pose to OpModeStorage & SpectrixTeleOp
                instant(() -> {
                    follower.stop(); // Stop motors at the end of auto!
                    Pose finalPose = follower.pose();
                    OpModeStorage.autonomousEndPose = finalPose;
                    SpectrixTeleOp.startingPose = finalPose;
                })
        );

        telemetry.addData("Status", "Ivy Autonomous Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        /*
         * Step 4: Wait for Driver to press PLAY on Driver Station.
         */
        waitForStart();

        if (isStopRequested()) return;

        /*
         * Step 5: Schedule the Ivy Command Sequence.
         */
        schedule(autoSequence);

        /*
         * Step 6: Main Execution Loop.
         * Scheduler.execute() processes active Ivy commands, checks completion,
         * and advances through the sequential command group.
         */
        while (opModeIsActive()) {
            Scheduler.execute(); // Drives the Ivy command scheduler!
            follower.update();   // Ensures localizer updates and motors stay stopped after sequence finishes!

            Pose pose = follower.pose();
            telemetry.addData("Ivy Scheduler", "Running Commands...");
            telemetry.addData("Position", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            telemetry.addData("Velocity", follower.velocity());
            telemetry.update();
        }
    }

    /**
     * Helper function to build an Ivy Command that follows a Pedro Pathing Path.
     *
     * @param follower The Follower instance to drive
     * @param path The Path to follow
     * @return An Ivy Command that executes path following until arrival
     */
    private Command followPath(Follower follower, Path path) {
        return Command.build()
                .setStart(() -> follower.follow(path))
                .setExecute(follower::update)
                .setDone(() -> !follower.isBusy())
                .requiring(follower);
    }

    /**
     * Helper function to build an Ivy Command that pauses for a duration in milliseconds
     * while continuously running follower.update() to keep localizer and motor holding active.
     *
     * @param follower The Follower instance
     * @param milliseconds Time to wait in milliseconds
     * @return An Ivy Command that waits while updating the follower
     */
    private Command waitMsWithUpdate(Follower follower, double milliseconds) {
        return race(
                waitMs(milliseconds),
                Command.build()
                        .setExecute(follower::update)
                        .requiring(follower)
        );
    }
}
