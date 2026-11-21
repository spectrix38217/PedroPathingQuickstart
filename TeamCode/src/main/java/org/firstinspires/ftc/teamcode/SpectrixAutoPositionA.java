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
        ballMechanism.setTargetShootRpm(4600.0); // Launch at 4500 RPM for Position A

        Pose startPose = new Pose(0, 0, 0); // Starting position for Position A
        follower.setPose(startPose);

        // Path 1: Drive to shooting position: X: 33.39 in, Y: -10.87 in, Heading: 70°
        Pose shootPose = new Pose(33.39, -10.87, Math.toRadians(62.5));
        Path pathShoot = line(startPose, shootPose).linear(0, Math.toRadians(62.5));

        // Path 2: Drive from shoot position to waypoint (7, 40)
        Pose waypointPose = new Pose(7, 40, Math.toRadians(25.0));
        Path pathWaypoint = line(shootPose, waypointPose).linear(Math.toRadians(70.0), Math.toRadians(25.0));

        // Path 3: Drive from waypoint to handoff position: X: 4.0 in, Y: 80.0 in, Heading: 0°
        Pose handoffPose = new Pose(10, 60, 0);
        Path pathHandoff = line(waypointPose, handoffPose).linear(Math.toRadians(25.0), 0);

        telemetry.addData("Status", "SpectrixAuto Position A Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- Action 1: Drive to shooting position (33.39, -10.87) @ 70° ---
        followPath(pathShoot, "1/4: Driving to Shoot Position (33.39, -10.87) @ 70°");

        // --- Action 2: Shoot 4 times (runs shooter flywheel & feeds balls for 4.0 seconds) ---
        shootBalls(4.0, "2/4: Shooting 4 Balls");

        // --- Action 3: Drive to waypoint (7, 40) ---
        followPath(pathWaypoint, "3/4: Driving to Waypoint (7, 40)");

        // --- Action 4: Drive to handoff position (4, 80) @ 0° heading ---
        followPath(pathHandoff, "4/4: Driving to Handoff (4, 80)");

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
}
