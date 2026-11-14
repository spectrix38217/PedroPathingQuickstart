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
 * 1. Drive to position (12, 30) facing 30° heading.
 * 2. Shoot 4 times (ramp shooter to target RPM, open stopper, feed balls).
 * 3. Drive to (-48, 0) facing 0° heading and hand off.
 * 4. Save final pose to OpModeStorage & SpectrixTeleOp.startingPose upon completion.
 */
@Autonomous(name = "SpectrixAuto Position A", group = "Autonomous")
public class SpectrixAutoPositionA extends LinearOpMode {

    private Follower follower;
    private BallMechanism ballMechanism;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        ballMechanism = new BallMechanism(hardwareMap);

        Pose startPose = new Pose(0, 0, 0); // Starting position for Position A
        follower.setPose(startPose);

        // Path 1: Drive to shooting position: X: 12 in, Y: 30 in, Heading: 30°
        Pose shootPose = new Pose(12, 30, Math.toRadians(30));
        Path pathShoot = line(startPose, shootPose).linear(0, Math.toRadians(30));

        // Path 2: Drive to handoff position: X: -48 in, Y: 0 in, Heading: 0°
        Pose handoffPose = new Pose(-48, 0, 0);
        Path pathHandoff = line(shootPose, handoffPose).linear(Math.toRadians(30), 0);

        telemetry.addData("Status", "SpectrixAuto Position A Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- Action 1: Drive to shooting position (12, 30) @ 30° ---
        followPath(pathShoot, "1/3: Driving to Shoot Position (12, 30) @ 30°");

        // --- Action 2: Shoot 4 times (runs shooter flywheel & feeds balls for 3.0 seconds) ---
        shootBalls(3.0, "2/3: Shooting 4 Balls");

        // --- Action 3: Drive to (-48, 0) @ 0° heading and hand off ---
        followPath(pathHandoff, "3/3: Driving to (-48, 0) @ 0° Handoff Position");

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
