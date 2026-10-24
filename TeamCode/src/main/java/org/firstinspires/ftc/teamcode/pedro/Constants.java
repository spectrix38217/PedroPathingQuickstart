package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
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
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(-9.947949507105069);
        c.yPodOffset.set(0.6709412702425259);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.12253180565955928);
                Controller secondaryTranslationalForward = Controller.proportional(0.04527222808105978);
                Controller primaryTranslationalLateral = Controller.proportional(0.16119417752197324);
                Controller secondaryTranslationalLateral = Controller.proportional(0.05955694140661912);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.007006429149217986));
                c.brake.set(Controller.proportionalFeedforward(0.005955464776835288));

                c.headingFeedback.set(Controller.proportional(2.373116656474042));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.049723310398500704, 0.011311785626704277));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06318026603089268, 0.008357239321500122));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014769400585873686, 0.002145944530634485));

                c.maxAchievableForwardVelocity.set(119.69289804746136);
                c.maxAchievableStrafeVelocity.set(89.00904393145653);
                c.naturalForwardDeceleration.set(36.39567513903826);
                c.naturalStrafeDeceleration.set(54.40328775185579);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}