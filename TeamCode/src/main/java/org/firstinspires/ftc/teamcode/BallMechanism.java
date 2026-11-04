package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BallMechanism - Encapsulates mechanism servos and non-drivetrain motors:
 * - "barLeft" & "barRight" (Servos in servo mode for flower ball release)
 * - "feeder" (CRServo in continuous mode for feeding shooter)
 * - "transfer" (DcMotor for intake & outtake transfer)
 */
public class BallMechanism {
    private final Servo barLeft;
    private final Servo barRight;
    private final CRServo feeder;
    private final DcMotor transfer;

    private boolean barIsDown = false;

    public BallMechanism(HardwareMap hardwareMap) {
        barLeft = hardwareMap.get(Servo.class, Constants.Servos.BAR_LEFT_NAME);
        barRight = hardwareMap.get(Servo.class, Constants.Servos.BAR_RIGHT_NAME);
        feeder = hardwareMap.get(CRServo.class, Constants.Servos.FEEDER_NAME);
        transfer = hardwareMap.get(DcMotor.class, Constants.Motors.TRANSFER_NAME);

        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize to neutral/stopped state
        barUp();
        stop();
    }

    /**
     * Bar Up (Neutral position): Sets both barLeft and barRight servos to 0.5.
     */
    public void barUp() {
        barLeft.setPosition(Constants.Servos.BAR_NEUTRAL_POS);
        barRight.setPosition(Constants.Servos.BAR_NEUTRAL_POS);
        barIsDown = false;
    }

    /**
     * Bar Down: Lowers bar to release balls from flowers.
     * Servos move in opposite directions due to mirrored mounting:
     * - barRight goes to positive offset from 0.5 (0.5 + 0.25 = 0.75)
     * - barLeft goes to negative offset from 0.5 (0.5 - 0.25 = 0.25)
     */
    public void barDown() {
        barRight.setPosition(Constants.Servos.BAR_NEUTRAL_POS + Constants.Servos.BAR_OFFSET);
        barLeft.setPosition(Constants.Servos.BAR_NEUTRAL_POS - Constants.Servos.BAR_OFFSET);
        barIsDown = true;
    }

    /**
     * Toggle Bar Position (Up <-> Down).
     */
    public void toggleBar() {
        if (barIsDown) {
            barUp();
        } else {
            barDown();
        }
    }

    /**
     * Intake: Collects balls from field by running "transfer" motor at full speed.
     * "feeder" servo holds (stops) to avoid passing balls into shooter.
     */
    public void intake() {
        transfer.setPower(Constants.Motors.TRANSFER_INTAKE_POWER);
        feeder.setPower(Constants.Servos.FEEDER_HOLD_POWER);
    }

    /**
     * Outtake: Spits out balls by running "transfer" motor backwards.
     * "feeder" servo holds (stops) during this operation.
     */
    public void outtake() {
        transfer.setPower(Constants.Motors.TRANSFER_OUTTAKE_POWER);
        feeder.setPower(Constants.Servos.FEEDER_HOLD_POWER);
    }

    /**
     * Transfer to Shoot: Runs "transfer" motor at full speed and "feeder" CRServo
     * at full speed in reverse to feed balls into shooter.
     */
    public void transferToShoot() {
        transfer.setPower(Constants.Motors.TRANSFER_INTAKE_POWER);
        feeder.setPower(Constants.Servos.FEEDER_SHOOT_POWER);
    }

    /**
     * Stops both transfer motor and feeder CRServo.
     */
    public void stop() {
        transfer.setPower(Constants.Motors.TRANSFER_STOP_POWER);
        feeder.setPower(Constants.Servos.FEEDER_HOLD_POWER);
    }

    // Getters for telemetry and state monitoring
    public boolean isBarDown() {
        return barIsDown;
    }

    public double getBarLeftPosition() {
        return barLeft.getPosition();
    }

    public double getBarRightPosition() {
        return barRight.getPosition();
    }

    public double getFeederPower() {
        return feeder.getPower();
    }

    public double getTransferPower() {
        return transfer.getPower();
    }
}
