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
 * SpectrixAutoPositionA - Autonomous OpMode for Position A starting location.
 *
 * <p>Execution Flow:
 * 1. Drive to shooting position (33.39, -10.87) facing 70° heading.
 * 2. Shoot 4 times at 4500 RPM.
 * 3. Drive to waypoint (7, 40).
 * 4. Drive to final handoff position (4, 80) facing 0° heading and hand off.
 * 5. Save final pose to OpModeStorage & SpectrixTeleOp.startingPose upon completion.
 */
@SuppressWarnings("unused")
@Autonomous(name = "SpectrixAuto Position A", group = "Autonomous")
public class SpectrixAutoPositionA extends LinearOpMode {

    private Follower follower;
    private BallMechanism ballMechanism;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        ballMechanism = new BallMechanism(hardwareMap);
        ballMechanism.setTargetShootRpm(4500.0); // Launch at 4500 RPM for Position A

        Pose startPose = new Pose(0, 0, 0); // Starting position for Position A
        follower.setPose(startPose);

        // Path 1: Drive to shooting position: X: 33.39 in, Y: -10.87 in, Heading: 62.5°
        Pose shootPose = new Pose(33.39, -10.87, Math.toRadians(62.5));
        Path pathShoot = line(startPose, shootPose).linear(0, Math.toRadians(62.5));

        // Path 2: Drive from shoot position to (19.53, 17.91)
        Pose intermediate1 = new Pose(19.53, 17.91, Math.toRadians(180));
        Path pathIntermediate1 = line(shootPose, intermediate1).linear(Math.toRadians(62.5), Math.toRadians(180));

        // Path 3: Drive to intake (11.3, 18.19) at 0.25 speed
        Pose intakePose = new Pose(11.3, 16.5, Math.toRadians(180));
        Path pathIntake = line(intermediate1, intakePose).linear(Math.toRadians(180), Math.toRadians(180))
                .with(Constants.foresightConfig.maxPathSpeed.at(0.25));

        // Path 4: Drive back to (19.53, 17.91)
        Path pathBackToIntermediate1 = line(intakePose, intermediate1).linear(Math.toRadians(180), Math.toRadians(180));

        // Path 5: Drive to handoff position: X: 10.0 in, Y: 60.0 in, Heading: 0°
        Pose handoffPose = new Pose(10, 66, 0);
        Path pathHandoff = line(intermediate1, handoffPose).linear(Math.toRadians(180), 0);

        telemetry.addData("Status", "SpectrixAuto Position A Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- Action 1: Drive to shooting position (33.39, -10.87) @ 62.5° ---
        followPath(pathShoot, "1/9: Driving to Shoot Position (33.39, -10.87) @ 62.5°");

        // Hold position during shooting to resist recoil
        follower.hold(shootPose);

        // --- Action 2: Shoot 4 times (runs shooter flywheel & feeds balls for 4.0 seconds) ---
        shootBalls(4.0, "2/9: Shooting 4 Balls");

        // --- Action 3: Drive to intermediate1 (19.53, 17.91) @ 180° ---
        followPath(pathIntermediate1, "3/9: Driving to (19.53, 17.91)");

        // --- Action 4: Lower bar, turn intake on, wait 1 second for bar to lower ---
        follower.hold(intermediate1);
        ballMechanism.barDown();
        ballMechanism.startIntake();
        waitSeconds(1.0, "4/9: Waiting for bar to lower");

        // --- Action 5: Drive to intake at 0.25 speed ---
        followPath(pathIntake, "5/9: Driving to intake (11.3, 18.19) at 0.25 speed");

        // --- Action 6: Stay still for 5 seconds ---
        follower.hold(intakePose);
        waitSeconds(5.0, "6/9: Intaking for 5 seconds");
        ballMechanism.idle(); // Turn intake off

        // --- Action 7: Drive back to intermediate1 and raise bar ---
        followPath(pathBackToIntermediate1, "7/9: Driving back to (19.53, 17.91)");
        ballMechanism.barUp();

        // --- Action 8: Drive to handoff position (10, 60) @ 0° heading ---
        followPath(pathHandoff, "8/9: Driving to Handoff (10, 60)");

        // --- Action 9: Ensure we are facing 0° ---
        follower.hold(new Pose(10, 66, 0));
        waitSeconds(1.5, "9/9: Holding 0° heading at handoff position");

        follower.stop();
        ballMechanism.idle();

        Pose finalPose = follower.pose();
        OpModeStorage.autonomousEndPose = finalPose;
        SpectrixTeleOp.startingPose = finalPose;

        while (opModeIsActive()) {
            follower.update();
            ballMechanism.update();

            telemetry.addData("Status", "Autonomous Position A Complete!");
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
     * Helper function to execute shooting sequence for a given duration.
     */
    @SuppressWarnings({"SameParameterValue", "unused"})
    private void shootBalls(double durationSeconds, String stepName) {
        ballMechanism.startShooting();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < durationSeconds) {
            follower.update();
            ballMechanism.update();

            Pose pose = follower.pose();
            telemetry.addData("Step", stepName);
            telemetry.addData("Shooting Progress", String.format(Locale.US, "%.1f / %.1f sec", timer.seconds(), durationSeconds));
            telemetry.addData("Shooter RPM", String.format(Locale.US, "%.0f / %.0f RPM", ballMechanism.getCurrentShooterRpm(), ballMechanism.getTargetShootRpm()));
            telemetry.addData("Position", String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            telemetry.update();
        }
        ballMechanism.idle();
    }

    /**
     * Helper function to pause execution for a given duration in seconds.
     * Keeps follower.update() running during pauses so odometry tracking remains active.
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
