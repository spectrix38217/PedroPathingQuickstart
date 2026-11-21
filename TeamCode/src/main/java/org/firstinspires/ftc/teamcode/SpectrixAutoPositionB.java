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
 *
 * <p>Execution Flow:
 * 1. Drive to shooting position (42.17, -51.38) facing 73.26° heading.
 * 2. Hold still and shoot for 4 seconds.
 * 3. Drive to (27.64, -15.28) facing 180° heading.
 * 4. Hold still for 1 second; during that lower the bar and turn on the intake.
 * 5. Drive to (10.87, -15.67) facing 180° heading at 0.25 speed.
 * 6. Hold for 4 seconds, then turn off the intake.
 * 7. Drive back to (27.64, -15.28) facing 180° heading.
 * 8. Raise bar.
 * 9. Drive to (7.13, 34.09) facing 0° heading.
 * 10. When robot is still, hand off position to teleop.
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
        ballMechanism.setTargetShootRpm(4750); // Set shooting RPM

        Pose startPose = new Pose(0, 0, 0); // Starting position for Position B
        follower.setPose(startPose);

        // Path 1: Drive to shooting position: X: 42.17 in, Y: -51.38 in, Heading: 73.26°
        Pose shootPose = new Pose(42.17, -51.38, Math.toRadians(73.26));
        Path pathShoot = line(startPose, shootPose).linear(0, Math.toRadians(73.26));

        // Path 2: Drive to intermediate pose: X: 27.64 in, Y: -15.28 in, Heading: 180°
        Pose intermediatePose = new Pose(27.64, -15.28, Math.toRadians(180));
        Path pathIntermediate = line(shootPose, intermediatePose).linear(Math.toRadians(73.26), Math.toRadians(180));

        // Path 3: Drive to intake pose: X: 10.87 in, Y: -15.67 in, Heading: 180° at 0.25 speed
        Pose intakePose = new Pose(10.87, -15.67, Math.toRadians(180));
        Path pathIntake = line(intermediatePose, intakePose).linear(Math.toRadians(180), Math.toRadians(180))
                .with(Constants.foresightConfig.maxPathSpeed.at(0.25));

        // Path 4: Drive back to intermediate pose: X: 27.64 in, Y: -15.28 in, Heading: 180°
        Path pathBack = line(intakePose, intermediatePose).linear(Math.toRadians(180), Math.toRadians(180));

        // Path 5: Drive to handoff position: X: 7.13 in, Y: 34.09 in, Heading: 0°
        Pose handoffPose = new Pose(7.13, 34.09, Math.toRadians(0));
        Path pathHandoff = line(intermediatePose, handoffPose).linear(Math.toRadians(180), Math.toRadians(0));

        telemetry.addData("Status", "SpectrixAuto Position B Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- Action 1: Drive to shooting position (42.17, -51.38) @ 73.26° ---
        followPath(pathShoot, "1/10: Driving to Shoot Position (42.17, -51.38) @ 73.26°");

        // Hold position during shooting to resist recoil
        follower.hold(shootPose);

        // --- Action 2: Hold still and shoot for 4 seconds ---
        shootBalls(4.0, "2/10: Shooting 4 Balls");

        // --- Action 3: Drive to (27.64, -15.28) @ 180° ---
        followPath(pathIntermediate, "3/10: Driving to (27.64, -15.28) @ 180°");

        // --- Action 4: Hold still for 1 second; during that lower the bar and turn on the intake ---
        follower.hold(intermediatePose);
        ballMechanism.barDown();
        ballMechanism.startIntake();
        waitSeconds(1.0, "4/10: Lowering bar & starting intake");

        // --- Action 5: Drive to (10.87, -15.67) @ 180° at 0.25 speed ---
        followPath(pathIntake, "5/10: Driving to intake (10.87, -15.67) at 0.25 speed");

        // --- Action 6: Hold for 4 seconds, then turn off the intake ---
        follower.hold(intakePose);
        waitSeconds(4.0, "6/10: Intaking for 4 seconds");
        ballMechanism.idle(); // Turn off intake

        // --- Action 7: Drive back to (27.64, -15.28) @ 180° ---
        followPath(pathBack, "7/10: Driving back to (27.64, -15.28)");

        // --- Action 8: Raise bar ---
        ballMechanism.barUp();
        waitSeconds(0.5, "8/10: Waiting for bar to raise");

        // --- Action 9: Drive to (7.13, 34.09) @ 0° heading ---
        followPath(pathHandoff, "9/10: Driving to Handoff (7.13, 34.09) @ 0°");

        // --- Action 10: When robot is still, hand off position to teleop ---
        follower.hold(handoffPose);
        waitSeconds(1.5, "10/10: Holding final position at handoff");

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
