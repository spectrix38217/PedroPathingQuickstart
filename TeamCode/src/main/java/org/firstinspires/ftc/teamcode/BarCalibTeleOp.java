package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

/**
 * BarCalibTeleOp - Dedicated Calibration OpMode for tuning the "stopper" Servo.
 *
 * <p>Controls:
 * - Y Button: Reset stopper servo position to 0.5 always.
 * - D-Pad Up / Down: Fine adjust position (+/- 0.01).
 * - D-Pad Right / Left: Coarse adjust position (+/- 0.05).
 *
 * <p>Telemetry outputs the exact stopper servo position to paste into Constants!
 */
@TeleOp(name = "Stopper Calibration", group = "TeleOp")
public class BarCalibTeleOp extends OpMode {

    private Servo stopper;
    private double position = 0.5;

    private boolean previousY = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;

    @Override
    public void init() {
        stopper = hardwareMap.get(Servo.class, "stopper");
        position = 0.5;
        stopper.setPosition(position);

        telemetry.addData("Status", "Stopper Calibration Initialized");
        telemetry.addData("Initial Position", position);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Edge detection for controls
        boolean yPressed = gamepad1.y && !previousY;
        boolean dpadUpPressed = gamepad1.dpad_up && !previousDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !previousDpadDown;
        boolean dpadLeftPressed = gamepad1.dpad_left && !previousDpadLeft;
        boolean dpadRightPressed = gamepad1.dpad_right && !previousDpadRight;

        // Y Button: Reset to 0.5 always
        if (yPressed) {
            position = 0.5;
        }

        // Fine adjustment (+/- 0.01) via D-Pad Up / Down
        if (dpadUpPressed) {
            position = Math.min(1.0, position + 0.01);
        }
        if (dpadDownPressed) {
            position = Math.max(0.0, position - 0.01);
        }

        // Coarse adjustment (+/- 0.05) via D-Pad Right / Left
        if (dpadRightPressed) {
            position = Math.min(1.0, position + 0.05);
        }
        if (dpadLeftPressed) {
            position = Math.max(0.0, position - 0.05);
        }

        // Apply position to stopper servo
        stopper.setPosition(position);

        previousY = gamepad1.y;
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;

        // Telemetry Display
        telemetry.addData("=== STOPPER SERVO CALIBRATION ===", "");
        telemetry.addData("Stopper Servo Position", String.format(Locale.US, "%.3f", position));
        telemetry.addLine("");
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Y Button         : Reset Position to 0.5");
        telemetry.addLine("D-Pad Up / Down  : Fine Adjust Position (+/- 0.01)");
        telemetry.addLine("D-Pad Right/Left : Coarse Adjust Position (+/- 0.05)");
        telemetry.update();
    }
}
