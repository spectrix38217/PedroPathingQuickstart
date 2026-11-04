package org.firstinspires.ftc.teamcode.states;

import static com.pedropathing.ivy.Scheduler.cancel;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.MecanumHeadingLock;
import org.firstinspires.ftc.teamcode.drivetrains.SwerveHeadingLock;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.shooter.Turret;

public class IntakingState implements State {

//    private static double dtLength = 276.948;
//    private static double dtWidth = 346.000;
    Pose redResetPoseFar = new Pose(10.48422986, 7.360112479, Math.toRadians(180));

    Pose redResetPoseNear = new Pose(123.67967981, 77.472471702756, Math.toRadians(0));



    public static double tickingIncrementInches = 2;

    Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private Command joystickToIntake;

//    private Command transition;

    private Command updateShooter;

    private boolean transitioning = false;

    public IntakingState(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void initialize(Robot robot, State prevState) {
        schedule(robot.closeGate());
//        schedule(robot.deactivateFlywheel());
        schedule(robot.activateShooter());
        schedule(robot.setPtoToIntaking());

//        transition = bind(() -> gamepad1.a).and(() -> !Constants.lastOpModeWasAuto).and(() -> !transitioning)
//                .rise(
//                        sequential(
//                                instant(() -> transitioning = true),
//                                instant(() -> cancel(transition)),
//                                robot.setIntakePower(0),
//                                instant(() -> robot.setState(States.SHOOTING))
//                        )
//                );

        updateShooter = robot.updateShootingSubsystems();


        joystickToIntake = robot.joysticksToIntakePower(
                () -> gamepad1.left_trigger,
                () -> gamepad1.right_trigger
        );

//        updateTurret = robot.updateTurret();

        if (!Constants.lastOpModeWasAuto) {
            schedule(joystickToIntake);
//            schedule(updateTurret);
            schedule(updateShooter);
        }

        transitioning = false;
    }

    public void execute(Robot robot) {
        //length = 15.39
        //width = 15.12

//        if (!Constants.lastOpModeWasAuto && gamepad1.y && gamepad1.dpad_up && gamepad2.y && gamepad2.dpad_up) {
//            transitioning = true;
//            cancel(joystickToIntake);
//            cancel(updateShooter);
//            schedule(
//                    instant(() -> robot.setState(States.LIFTING))
//            );
//        }

        if(robot.beamBroken()) {
            gamepad1.rumble(50);
        }


        if (gamepad2.leftBumperWasPressed()) {
            Constants.debugTelemetry = !Constants.debugTelemetry;
        }

        if (gamepad2.aWasPressed())  {

            if(robot.getInvertedDrive()) {
                robot.setPose((Constants.color == Constants.Color.RED) ? redResetPoseFar : redResetPoseFar.mirror());
            } else {
                robot.setPose((Constants.color == Constants.Color.RED) ? redResetPoseNear : redResetPoseNear.mirror());
            }

            robot.resetGoalPose();
        }


        if (gamepad1.rightBumperWasPressed() && !Constants.lastOpModeWasAuto && !transitioning) {
            transitioning = true;
            cancel(joystickToIntake);
            cancel(updateShooter);
            schedule(
                    robot.setIntakePower(0),
                    instant(() -> robot.setState(States.SHOOTING))
            );
        }

        if (gamepad1.leftBumperWasPressed() && !Constants.lastOpModeWasAuto && !transitioning) {
            transitioning = true;
            schedule(sequential(
                    instant(() -> {
                        cancel(joystickToIntake);
                    }),
                    robot.shoot(),
                    instant(() -> transitioning = false),
                    instant(() -> {
                        schedule(joystickToIntake);
                    })
            ));
        }

        if (gamepad1.aWasPressed() && !Constants.lastOpModeWasAuto && robot.getDrivetrain() instanceof MecanumHeadingLock) {
            double heading = Constants.color == Constants.Color.RED
                    ? Math.toRadians(32)
                    : Math.toRadians(148);
            ((MecanumHeadingLock) robot.getDrivetrain()).setTargetHeading(heading);
        }

        if (gamepad1.bWasPressed() && !Constants.lastOpModeWasAuto && robot.getDrivetrain() instanceof MecanumHeadingLock) {
            double heading = Constants.color == Constants.Color.RED
                    ? Math.toRadians(180)
                    : Math.toRadians(0);
            ((MecanumHeadingLock) robot.getDrivetrain()).setTargetHeading(heading);
        }


        if (gamepad1.yWasPressed() && !Constants.lastOpModeWasAuto && robot.getDrivetrain() instanceof MecanumHeadingLock) {
            double heading = Constants.color == Constants.Color.RED
                    ? Math.toRadians(0)
                    : Math.toRadians(180);
            ((MecanumHeadingLock) robot.getDrivetrain()).setTargetHeading(heading);
        }


        if (gamepad1.xWasPressed() && !Constants.lastOpModeWasAuto && robot.getDrivetrain() instanceof MecanumHeadingLock) {
            double heading = Constants.color == Constants.Color.RED
                    ? Math.toRadians(-35)
                    : Math.toRadians(210);
            ((MecanumHeadingLock) robot.getDrivetrain()).setTargetHeading(heading);
        }


//        if ((gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) && !Constants.lastOpModeWasAuto) {
//            Turret.turretOffsetRad += Math.toRadians(3);
//        }
//
//        if ((gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) && !Constants.lastOpModeWasAuto) {
//            Turret.turretOffsetRad -= Math.toRadians(3);
//        }

        if(gamepad1.dpadUpWasPressed()) {
            robot.setInvertedDrive(false);
        }

        if(gamepad1.dpadDownWasPressed()) {
            robot.setInvertedDrive(true);
        }



        if ((gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) && !Constants.lastOpModeWasAuto) {
            robot.moveGoalPose(0,
                    (Constants.color == Constants.Color.RED) ? tickingIncrementInches : -tickingIncrementInches);
        }

        if ((gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) && !Constants.lastOpModeWasAuto) {
            robot.moveGoalPose(0,
                    (Constants.color == Constants.Color.RED) ? -tickingIncrementInches : tickingIncrementInches);
        }

        if ((/*gamepad1.dpadUpWasPressed() || */gamepad2.dpadUpWasPressed()) && !Constants.lastOpModeWasAuto) {
            robot.moveGoalPose((Constants.color == Constants.Color.RED) ? tickingIncrementInches : -tickingIncrementInches
                    ,0);
        }

        if ((/*gamepad1.dpadDownWasPressed() || */gamepad2.dpadDownWasPressed()) && !Constants.lastOpModeWasAuto) {
            robot.moveGoalPose((Constants.color == Constants.Color.RED) ? -tickingIncrementInches : tickingIncrementInches
                    ,0);
        }
    }

    public String name(){
        return "Intaking";
    }
}
