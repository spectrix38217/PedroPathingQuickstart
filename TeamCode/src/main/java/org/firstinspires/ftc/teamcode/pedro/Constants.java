package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    /* chudbot
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });   */

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    /* chudbot
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(0);
        c.yPodPort.set(1);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-0.0);
        c.yPodOffset.set(-0.01968503937007874);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0029082363709239);
    }); */

//    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
//        c.name.set("octoquad");
//        c.xPodPort.set(0);
//        c.yPodPort.set(1);
//        c.ticksPerUnit.set(505.316944406);
//        c.xPodOffset.set(-1.1023622047244095);
//        c.yPodOffset.set(0.767716535433071);
//        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
//        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
//        c.headingScalar.set(1.0097679269820663);
//    });

    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(0);
        c.yPodPort.set(1);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(6.181102362204725);
        c.yPodOffset.set(1.5748031496062993);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_2_M1_PLUS_SCL_IDLE_ONESHOT_TGL);
        c.headingScalar.set(1.0095375423565784);
    });

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.23383045429363739);
//                Controller secondaryTranslationalForward = Controller.proportional(0.08639410479668803);
//                Controller primaryTranslationalLateral = Controller.proportional(0.318670925643722);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.11774039198139828);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.009864046726247242));
//                c.brake.set(Controller.proportionalFeedforward(0.008384439717310156));
//
//                c.headingFeedback.set(Controller.proportional(3.0749175974993683));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06741245521755912, 0.00920443775016025));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.05637595951731939, 0.06042487205764109));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002974695651846195, 0.0031578411591617686));
//
//                c.maxAchievableForwardVelocity.set(89.14539165651483);
//                c.maxAchievableStrafeVelocity.set(76.52309304693615);
//                c.naturalForwardDeceleration.set(17.450772847206967);
//                c.naturalStrafeDeceleration.set(20.296567510697628);
//            }
//    );
public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            // Lowered proportional gains by ~30% and added small D-gains to stop oscillation/shaking
            Controller primaryTranslationalForward = Controller.pid(0.15, 0.0, 0.015);
            Controller secondaryTranslationalForward = Controller.pid(0.05, 0.0, 0.005);
            Controller primaryTranslationalLateral = Controller.pid(0.18, 0.0, 0.018);
            Controller secondaryTranslationalLateral = Controller.pid(0.07, 0.0, 0.007);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.00986822958870424));
            c.brake.set(Controller.proportionalFeedforward(0.008387995150398604));

            c.headingFeedback.set(Controller.pid(2.5, 0.0, 0.1));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06101420673066478, 0.005836027332170973));

            c.linearBrakeCoefficients.set(Matrix.diag(0.09312318256846111, 0.053350517226596235));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013112951645084827, 0.0018857567129918003));

            c.maxAchievableForwardVelocity.set(93.36589528234913);
            c.maxAchievableStrafeVelocity.set(78.47502346539679);
            c.naturalForwardDeceleration.set(23.267619310797713);
            c.naturalStrafeDeceleration.set(47.93557985135556);
        }
);

    // Servo Hardware Mappings & Config Constants
    public static class Servos {
        public static String BAR_LEFT_NAME = "barLeft";
        public static String BAR_RIGHT_NAME = "barRight";
        public static String FEEDER_NAME = "feeder";
        public static String STOPPER_NAME = "stopper";

        // Bar neutral position & offset for mirrored servos
        public static double BAR_NEUTRAL_POS = 0.5;
        public static double BAR_OFFSET = 0.4; // right = 0.5 + 0.4 (0.9), left = 0.5 - 0.4 (0.1)

        // Stopper Servo Positions
        public static double STOPPER_NEUTRAL_POS = 0.3;
        public static double STOPPER_OPEN_POS = 0.5;
        public static double STOPPER_CLOSED_POS = 0.3;

        // Feeder CRServo Power Levels (-1.0 to 1.0)
        public static double FEEDER_SHOOT_POWER = -0.5; // Slowed down from -1.0 so balls aren't fed too fast
        public static double FEEDER_HOLD_POWER = 0.0;  // Holding position (stopped)
        public static double FEEDER_FORWARD_POWER = 1.0;
    }

    // Motor Hardware Mappings & Config Constants
    public static class Motors {
        public static String TRANSFER_NAME = "transfer";
        public static String SHOOTER_NAME = "shooter";

        // Transfer Motor Power Levels
        public static double TRANSFER_INTAKE_POWER = 0.8;   // 80% speed intake
        public static double TRANSFER_OUTTAKE_POWER = -0.6; // 60% speed outtake (reverse)
        public static double TRANSFER_STOP_POWER = 0.0;

        // Shooter Motor & Velocity Config
        public static double SHOOTER_MAX_RPM = 5500.0;      // Physical motor max RPM at full 12-14V battery voltage
        public static double SHOOTER_IDLE_RPM = 2500.0;     // Predefined idle speed (2500 RPM)
        public static double SHOOTER_SHOOT_RPM = 4750.0;    // Target shooting speed (4600 RPM)
        public static double SHOOTER_RPM_TOLERANCE = 150.0;  // 150 RPM buffer window before stopper opens / re-stabilizes
        public static double SHOOTER_RAMP_TIMEOUT_SEC = 0.8; // Maximum wait time before forced feed
        public static double SHOOTER_TICKS_PER_REV = 28.0; // 1:1 encoder ticks per revolution
        public static double SHOOTER_P = 0.0004;          // Proportional gain for fast, active RPM correction
        public static double SHOOTER_I = 0.00005;         // Integral gain for steady-state RPM error
        public static double SHOOTER_FILTER_ALPHA = 0.25;  // Low-pass filter smoothing factor (0.0 to 1.0)
        public static double SHOOTER_MAX_BOOST_POWER = 0.20; // Maximum power boost above feedforward
    }

    public static Follower create(HardwareMap h) {
        return new Follower(
                new OctoQuadLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}