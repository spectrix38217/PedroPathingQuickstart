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
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.203199371578187);
        c.yPodOffset.set(-0.017317104058002863);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.21541394593837285);
                Controller secondaryTranslationalForward = Controller.proportional(0.07958969705758413);
                Controller primaryTranslationalLateral = Controller.proportional(0.2605982648373263);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0962840955435104);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010344060266617027));
                c.brake.set(Controller.proportionalFeedforward(0.008792451226624472));

                c.headingFeedback.set(Controller.proportional(2.872819167464649));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06831578605707352, 0.010520887541546705));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07523601141360965, 0.057573668336721125));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025767614064640116, 0.00300676656970448));

                c.maxPathSpeed.set(0.5); // Limit max autonomous path speed to 50%

                c.maxAchievableForwardVelocity.set(82.84083577335304);
                c.maxAchievableStrafeVelocity.set(66.96204192215029);
                c.naturalForwardDeceleration.set(22.13025903495807);
                c.naturalStrafeDeceleration.set(22.613656616478025);
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