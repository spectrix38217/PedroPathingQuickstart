package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BallMechanism - Encapsulates mechanism servos, motors, and state machine:
 * - "barLeft" & "barRight" (Servos in servo mode for flower ball release)
 * - "feeder" (CRServo in continuous mode for feeding shooter)
 * - "transfer" (DcMotor for intake & outtake transfer)
 * - "shooter" (DcMotorEx with encoder for high-speed shooter flywheel control)
 */
public class BallMechanism {

    public enum State {
        IDLE,     // Shooter @ 2500 RPM, Feeder @ 0, Transfer @ 0
        INTAKE,   // Shooter @ 2500 RPM, Feeder @ 0, Transfer @ 0.8
        OUTTAKE,  // Shooter @ 2500 RPM, Feeder @ 0, Transfer @ -0.6
        SHOOTING  // Shooter @ 5200 RPM, Feeder @ -1.0, Transfer Pulsed (0.5s off / 0.5s on)
    }

    private final Servo barLeft;
    private final Servo barRight;
    private final CRServo feeder;
    private final DcMotor transfer;
    private final DcMotorEx shooter;

    private State currentState = State.IDLE;
    private boolean barIsDown = false;

    private double targetShootRpm = Constants.Motors.SHOOTER_SHOOT_RPM;
    private double targetIdleRpm = Constants.Motors.SHOOTER_IDLE_RPM;
    private double targetShooterRpm = Constants.Motors.SHOOTER_IDLE_RPM;
    private double currentShooterRpm = 0.0;
    private double shooterPower = 0.0;

    private final ElapsedTime shootingTimer = new ElapsedTime();

    public BallMechanism(HardwareMap hardwareMap) {
        barLeft = hardwareMap.get(Servo.class, Constants.Servos.BAR_LEFT_NAME);
        barRight = hardwareMap.get(Servo.class, Constants.Servos.BAR_RIGHT_NAME);
        feeder = hardwareMap.get(CRServo.class, Constants.Servos.FEEDER_NAME);
        transfer = hardwareMap.get(DcMotor.class, Constants.Motors.TRANSFER_NAME);
        shooter = hardwareMap.get(DcMotorEx.class, Constants.Motors.SHOOTER_NAME);

        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // No brake needed when turning off

        // Initialize to neutral/idle state
        barUp();
        idle();
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
     * Transition State Machine to INTAKE.
     */
    public void startIntake() {
        if (currentState != State.INTAKE) {
            currentState = State.INTAKE;
        }
    }

    /**
     * Toggles Intake state ON <-> OFF.
     * If currently in INTAKE, switches to IDLE.
     * Otherwise, switches to INTAKE.
     */
    public void toggleIntake() {
        if (currentState == State.INTAKE) {
            currentState = State.IDLE;
        } else {
            currentState = State.INTAKE;
        }
    }

    /**
     * Transition State Machine to OUTTAKE.
     */
    public void startOuttake() {
        if (currentState != State.OUTTAKE) {
            currentState = State.OUTTAKE;
        }
    }

    /**
     * Transition State Machine to SHOOTING.
     * Resets shooting timer for initial feeder delay & pulsed transfer cycle.
     */
    public void startShooting() {
        if (currentState != State.SHOOTING) {
            currentState = State.SHOOTING;
            shootingTimer.reset();
        }
    }

    /**
     * Transition State Machine to IDLE.
     * Returns shooter to idle RPM, stops transfer and feeder.
     */
    public void idle() {
        if (currentState != State.IDLE) {
            currentState = State.IDLE;
        }
    }

    /**
     * Adjusts target shooter RPM up or down by the specified delta.
     * Modifies target shooting RPM if currently in SHOOTING state, otherwise modifies target idle RPM.
     *
     * @param deltaRpm Amount to change RPM by (e.g. +100.0 or -100.0).
     */
    public void adjustShooterRpm(double deltaRpm) {
        if (currentState == State.SHOOTING) {
            targetShootRpm = Math.min(Constants.Motors.SHOOTER_MAX_RPM, Math.max(1000.0, targetShootRpm + deltaRpm));
            setShooterSpeed(targetShootRpm);
        } else {
            targetIdleRpm = Math.min(Constants.Motors.SHOOTER_MAX_RPM, Math.max(0.0, targetIdleRpm + deltaRpm));
            setShooterSpeed(targetIdleRpm);
        }
    }

    /**
     * Sets target shooter speed in RPM.
     *
     * @param targetRpm Desired flywheel speed in RPM.
     */
    public void setShooterSpeed(double targetRpm) {
        this.targetShooterRpm = Math.min(Constants.Motors.SHOOTER_MAX_RPM, Math.max(0.0, targetRpm));
    }

    /**
     * Sets shooter to default idle speed (2500 RPM).
     */
    public void shooterIdle() {
        setShooterSpeed(targetIdleRpm);
    }

    /**
     * Completely turns off the shooter motor by cutting power (FLOAT mode, no brake).
     */
    public void shooterOff() {
        setShooterSpeed(0.0);
        shooterPower = 0.0;
        shooter.setPower(0.0);
    }

    /**
     * Updates the Mechanism State Machine, pulsed transfer logic, and shooter velocity loop.
     * MUST be called once per loop iteration in TeleOp / Auto.
     */
    public void update() {
        switch (currentState) {
            case INTAKE:
                setShooterSpeed(targetIdleRpm);
                transfer.setPower(Constants.Motors.TRANSFER_INTAKE_POWER);
                feeder.setPower(Constants.Servos.FEEDER_HOLD_POWER);
                break;

            case OUTTAKE:
                setShooterSpeed(targetIdleRpm);
                transfer.setPower(Constants.Motors.TRANSFER_OUTTAKE_POWER);
                feeder.setPower(Constants.Servos.FEEDER_HOLD_POWER);
                break;

            case SHOOTING:
                setShooterSpeed(targetShootRpm);
                feeder.setPower(Constants.Servos.FEEDER_SHOOT_POWER);

                double timeInShooting = shootingTimer.seconds();
                if (timeInShooting < Constants.Motors.TRANSFER_PULSE_DELAY_SEC) {
                    // Initial delay (0.5s): Feeder runs to prime shooter, transfer stays OFF
                    transfer.setPower(Constants.Motors.TRANSFER_STOP_POWER);
                } else {
                    // Pulsed transfer: ON for 0.5s, OFF for 0.5s repeatedly
                    double pulseCycleTime = (timeInShooting - Constants.Motors.TRANSFER_PULSE_DELAY_SEC)
                            % (Constants.Motors.TRANSFER_PULSE_ON_SEC + Constants.Motors.TRANSFER_PULSE_OFF_SEC);

                    if (pulseCycleTime < Constants.Motors.TRANSFER_PULSE_ON_SEC) {
                        transfer.setPower(Constants.Motors.TRANSFER_INTAKE_POWER);
                    } else {
                        transfer.setPower(Constants.Motors.TRANSFER_STOP_POWER);
                    }
                }
                break;

            case IDLE:
            default:
                setShooterSpeed(targetIdleRpm);
                transfer.setPower(Constants.Motors.TRANSFER_STOP_POWER);
                feeder.setPower(Constants.Servos.FEEDER_HOLD_POWER);
                break;
        }

        // Update flywheel velocity controller with low-pass filtering
        updateShooterSpeed();
    }

    /**
     * Monitors current RPM via encoder velocity, applies exponential low-pass filtering,
     * and calculates motor power using Feedforward + Smoothed Proportional error correction.
     */
    private void updateShooterSpeed() {
        if (targetShooterRpm <= 0.0) {
            currentShooterRpm = 0.0;
            shooterPower = 0.0;
            shooter.setPower(0.0);
            return;
        }

        // Calculate raw RPM from encoder velocity (ticks/sec)
        double ticksPerSec = shooter.getVelocity();
        double rawRpm = (ticksPerSec * 60.0) / Constants.Motors.SHOOTER_TICKS_PER_REV;

        // Low-pass exponential filter to eliminate high-frequency encoder tick noise
        currentShooterRpm = (currentShooterRpm * (1.0 - Constants.Motors.SHOOTER_FILTER_ALPHA))
                          + (rawRpm * Constants.Motors.SHOOTER_FILTER_ALPHA);

        // Feedforward + Capped Proportional Error Correction
        double ffPower = targetShooterRpm / Constants.Motors.SHOOTER_MAX_RPM;
        double rpmError = targetShooterRpm - currentShooterRpm;
        double pOutput = Math.min(0.20, Math.max(-0.20, rpmError * Constants.Motors.SHOOTER_P));

        shooterPower = Math.min(1.0, Math.max(0.0, ffPower + pOutput));
        shooter.setPower(shooterPower);
    }

    // Getters for telemetry and state monitoring
    public State getCurrentState() {
        return currentState;
    }

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

    public double getTargetShooterRpm() {
        return targetShooterRpm;
    }

    public double getCurrentShooterRpm() {
        return currentShooterRpm;
    }

    public double getShooterPower() {
        return shooterPower;
    }
}
