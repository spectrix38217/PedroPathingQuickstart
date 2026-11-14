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

        Pose pose1 = new Pose(24, 0, 0);
        Path path1 = line(startPose, pose1).constant(0);

        double ccw90Deg = Math.toRadians(90);
        Pose pose2 = new Pose(24, 12, ccw90Deg);
        Path path2 = line(pose1, pose2).linear(0, ccw90Deg);

        Path path3 = line(pose2, startPose).linear(ccw90Deg, 0);

        telemetry.addData("Status", "SpectrixAuto Position A Initialized & Ready!");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        followPath(path1, "1/3: Driving Forward 24 Inches");
        waitSeconds(1.0, "Waiting 1 Second");
        followPath(path2, "2/3: Strafing Left & Turning 90°");
        followPath(path3, "3/3: Returning to Origin (0,0)");

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
