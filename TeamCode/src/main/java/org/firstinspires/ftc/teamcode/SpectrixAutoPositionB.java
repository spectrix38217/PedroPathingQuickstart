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
 * SpectrixAutoPositionB - Autonomous OpMode for Position B starting location.
 */
@SuppressWarnings("unused")
@Autonomous(name = "SpectrixAuto Position B", group = "Autonomous")
public class SpectrixAutoPositionB extends LinearOpMode {

    private Follower follower;
    private BallMechanism ballMechanism;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        ballMechanism = new BallMechanism(hardwareMap);

        Pose startPose = new Pose(0, 0, 0); // Starting position for Position A
        follower.setPose(startPose);

        // Path 1: Drive to shooting position: X: 33.39 in, Y: -10.87 in, Heading: 70°
        Pose shootPose = new Pose(0, 6, Math.toRadians(90));
        Path pathShoot = line(startPose, shootPose).linear(0, Math.toRadians(90));

        Pose pose1 = new Pose(24, 0, 0);
        Path path1 = line(startPose, pose1).constant(0);

        double ccw90Deg = Math.toRadians(90);
        Pose pose2 = new Pose(24, 12, ccw90Deg);
        Path path2 = line(pose1, pose2).linear(0, ccw90Deg);

        Path path3 = line(pose2, startPose).linear(ccw90Deg, 0);

        telemetry.addData("Status", "SpectrixAuto Position B Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- Action 1: Drive to shooting position ---
        followPath(pathShoot, "1/4: Driving to Shoot Position");

        // Hold position during shooting to resist recoil
        follower.hold(shootPose);

        // --- Action 2: Shoot 4 times (runs shooter flywheel & feeds balls for 4.0 seconds) ---
        shootBalls(4.0, "2/4: Shooting 4 Balls");

        // --- Action 3: Drive to handoff position ---
        // Implement final return path or parking path as configured
        followPath(path3, "3/4: Driving to Handoff");

        // --- Action 4: Ensure we are facing correct heading at handoff ---
        follower.hold(startPose);
        waitSeconds(1.5, "4/4: Holding final position at handoff");

        follower.stop();
        ballMechanism.idle();

        Pose finalPose = follower.pose();
        OpModeStorage.autonomousEndPose = finalPose;
        SpectrixTeleOp.startingPose = finalPose;

        while (opModeIsActive()) {
            follower.update();
            ballMechanism.update();

            telemetry.addData("Status", "Autonomous Position B Complete!");
            telemetry.addData("Final Saved Pose", String.format(Locale.US, "X: %.2f in, Y: %.2f in, H: %.2f°", finalPose.x(), finalPose.y(), Math.toDegrees(finalPose.heading())));
            telemetry.update();
        }
    }

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

    @SuppressWarnings({"SameParameterValue", "unused"})
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
