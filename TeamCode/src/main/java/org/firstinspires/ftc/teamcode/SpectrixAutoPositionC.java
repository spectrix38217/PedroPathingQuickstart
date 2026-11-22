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
 * SpectrixAutoPositionC - Autonomous OpMode for Position C starting location.
 *
 * <p>Execution Flow:
 * 1. Start at (0, 126.26) facing 0° heading.
 * 2. Drive to shooting position (42.40, 118.30) facing 285.26° heading.
 * 3. Hold position for 7 seconds.
 * 4. Shoot for 4 seconds.
 * 5. Drive to intermediate position (42.44, 112.00) facing 90° heading.
 * 6. Stop for 1 second as the bar lowers and the intake turns on.
 * 7. Drive to intake position (43.19, 118.5) facing 90° heading at 0.25 speed.
 * 8. Stop for 3 seconds, then turn off intake.
 * 9. Drive backwards 8 inches to (43.19, 110.50) facing 90° heading.
 * 10. Raise bar.
 * 11. Drive back to shooting position (42.40, 118.30) facing 285.26° heading.
 * 12. Shoot for 4 seconds.
 * 13. Drive to handoff position (6.61, 106.18) facing 0° heading.
 * 14. Once robot stops (wait 1 sec), hand off position to teleop.
 */
@SuppressWarnings("unused")
@Autonomous(name = "SpectrixAuto Position C", group = "Autonomous")
public class SpectrixAutoPositionC extends LinearOpMode {

    private Follower follower;
    private BallMechanism ballMechanism;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        ballMechanism = new BallMechanism(hardwareMap);
        ballMechanism.setTargetShootRpm(4750); // Set target shooting RPM

        Pose startPose = new Pose(0, 126.26, Math.toRadians(0)); // Starting position for Position C
        follower.setPose(startPose);

        // Path 1: Drive to shooting position: X: 42.40 in, Y: 118.30 in, Heading: 285.26°
        Pose shootPose = new Pose(42.40, 118.30, Math.toRadians(285.26));
        Path pathShoot = line(startPose, shootPose).linear(Math.toRadians(0), Math.toRadians(285.26));

        // Path 2: Drive to intermediate position: X: 42.44 in, Y: 112.00 in, Heading: 90°
        Pose intermediatePose = new Pose(42.44, 112.00, Math.toRadians(90));
        Path pathIntermediate = line(shootPose, intermediatePose).linear(Math.toRadians(285.26), Math.toRadians(90));

        // Path 3: Drive to intake position: X: 43.19 in, Y: 118.5 in, Heading: 90° at 0.25 speed
        Pose intakePose = new Pose(43.19, 118.5, Math.toRadians(90));
        Path pathIntake = line(intermediatePose, intakePose).linear(Math.toRadians(90), Math.toRadians(90))
                .with(Constants.foresightConfig.maxPathSpeed.at(0.25));

        // Path 4: Drive backwards 8 inches to X: 43.19 in, Y: 110.50 in, Heading: 90°
        Pose backwardPose = new Pose(43.19, 110.50, Math.toRadians(90));
        Path pathBackward = line(intakePose, backwardPose).linear(Math.toRadians(90), Math.toRadians(90));

        // Path 5: Drive back to shooting position: X: 42.40 in, Y: 118.30 in, Heading: 285.26°
        Path pathBackToShoot = line(backwardPose, shootPose).linear(Math.toRadians(90), Math.toRadians(285.26));

        // Path 6: Drive to handoff position: X: 6.61 in, Y: 106.18 in, Heading: 0°
        Pose handoffPose = new Pose(6.61, 106.18, Math.toRadians(0));
        Path pathHandoff = line(shootPose, handoffPose).linear(Math.toRadians(285.26), Math.toRadians(0));

        telemetry.addData("Status", "SpectrixAuto Position C Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- Action 1: Drive to shooting position (42.40, 118.30) @ 285.26° ---
        followPath(pathShoot, "1/14: Driving to Shoot Position (42.40, 118.30) @ 285.26°");

        // Hold position for 5 seconds before shooting
        follower.hold(shootPose);
        waitSeconds(5.0, "2/14: Holding at shoot position for 7 seconds");

        // --- Action 2: Shoot for 4 seconds ---
        shootBalls(4.0, "3/14: Shooting for 4 seconds");

        // --- Action 3: Drive to (42.44, 112.00) @ 90° ---
        followPath(pathIntermediate, "4/14: Driving to (42.44, 112.00) @ 90°");

        // --- Action 4: Stop for 1 second as bar lowers and intake turns ON ---
        follower.hold(intermediatePose);
        ballMechanism.barDown();
        ballMechanism.startIntake();
        waitSeconds(1.0, "5/14: Lowering bar & starting intake");

        // --- Action 5: Drive to intake (43.19, 118.5) @ 90° at 0.25 speed ---
        followPath(pathIntake, "6/14: Driving to intake (43.19, 118.5) at 0.25 speed");

        // --- Action 6: Stop for 3 seconds, then turn off intake ---
        follower.hold(intakePose);
        waitSeconds(3.0, "7/14: Intaking for 3 seconds");
        ballMechanism.idle(); // Turn off intake

        // --- Action 7: Drive backwards 8 inches to (43.19, 110.50) @ 90° before raising bar ---
        followPath(pathBackward, "8/14: Driving backwards 8 in to (43.19, 110.50)");

        // --- Action 8: Raise bar ---
        follower.hold(backwardPose);
        ballMechanism.barUp();
        waitSeconds(0.5, "9/14: Raising bar");

        // --- Action 9: Drive back to shooting position (42.40, 118.30) @ 285.26° ---
        followPath(pathBackToShoot, "10/14: Driving back to shooting position (42.40, 118.30)");

        // --- Action 10: Hold at shoot position and shoot for 4 seconds ---
        follower.hold(shootPose);
        shootBalls(4.0, "11/14: Shooting for 4 seconds");

        // --- Action 11: Drive to handoff position (6.61, 106.18) @ 0° heading ---
        followPath(pathHandoff, "12/14: Driving to Handoff (6.61, 106.18) @ 0°");

        // --- Action 12: Once robot stops (1.0 second wait), hand off position to teleop ---
        follower.hold(handoffPose);
        waitSeconds(1.0, "13/14: Holding final position at handoff");

        follower.stop();
        ballMechanism.idle();

        Pose finalPose = follower.pose();
        OpModeStorage.autonomousEndPose = finalPose;
        SpectrixTeleOp.startingPose = finalPose;

        while (opModeIsActive()) {
            follower.update();
            ballMechanism.update();
            ballMechanism.getRgbLight().cyclePurpleCyan(0.5);

            telemetry.addData("Status", "Autonomous Position C Complete!");
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
