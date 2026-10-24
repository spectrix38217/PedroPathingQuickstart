package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Field Centric TeleOp", group = "TeleOp")
public class teleOp extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    // Default fallback pose if an autonomous didn't run first
    private final Pose defaultStartPose = p.of(24, 24, 0);

    @Override
    public void init() {
        // Initialize the follower using your Constants configuration
        follower = Constants.create(hardwareMap);

        // Attempt to load the end pose from Autonomous, fallback to default if null
        if (OpModeStorage.autonomousEndPose != null) {
            follower.setPose(OpModeStorage.autonomousEndPose);
        } else {
            follower.setPose(defaultStartPose);
        }
    }

    @Override
    public void start() {
        // No heavy logic needed here; initialization handles the pose setup
    }

    @Override
    public void loop() {
        // 1. Calculate Field-Centric drive powers using current robot heading
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        // 2. Pass those powers into the follower to execute them
        follower.manual(powers);

        // 3. Optional: Reset your pose manually via gamepad button (using custom Options if needed)
        if (gamepad1.options) {
            // Sets manual override to (10.5, 10.5) facing 90 degrees (stored internally as radians)
            follower.setPose(new Pose(10.5, 10.5, Math.toRadians(90)));
        }

        // 4. Update the follower state (Call this EXACTLY ONCE per loop)
        follower.update();

        // 5. Telemetry output
        Pose robotPose = follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading (Deg)", Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Save the final TeleOp position just in case you need it later
        OpModeStorage.autonomousEndPose = follower.pose();
    }
}