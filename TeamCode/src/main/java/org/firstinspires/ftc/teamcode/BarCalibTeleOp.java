package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Locale;

/**
 * BarCalibTeleOp - Dedicated Calibration OpMode for tuning the Flower Release Bar servos.
 *
 * <p>Controls:
 * - Y Button: Toggle Bar between Neutral UP (0.5) and Down (0.5 ± offset).
 * - D-Pad Up / Down: Fine adjust offset magnitude (+/- 0.01).
 * - D-Pad Right / Left: Coarse adjust offset magnitude (+/- 0.05).
 *
 * <p>Telemetry outputs the exact BAR_OFFSET value to paste into Constants.java!
 */
@TeleOp(name = "Bar Calibration", group = "TeleOp")
public class BarCalibTeleOp extends OpMode {

    private Servo barLeft;
    private Servo barRight;

    private double offset = Constants.Servos.BAR_OFFSET; // Default offset (0.25)
    private boolean isBarDown = false;

    private boolean previousY = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;

    @Override
    public void init() {
        barLeft = hardwareMap.get(Servo.class, Constants.Servos.BAR_LEFT_NAME);
        barRight = hardwareMap.get(Servo.class, Constants.Servos.BAR_RIGHT_NAME);

        offset = Constants.Servos.BAR_OFFSET;
        isBarDown = false;

        // Set servos to initial neutral position (0.5)
        barLeft.setPosition(Constants.Servos.BAR_NEUTRAL_POS);
        barRight.setPosition(Constants.Servos.BAR_NEUTRAL_POS);

        telemetry.addData("Status", "Bar Calibration Initialized");
        telemetry.addData("Default Neutral", Constants.Servos.BAR_NEUTRAL_POS);
        telemetry.addData("Initial Offset", offset);
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

        // Toggle Bar Position (Up <-> Down)
        if (yPressed) {
            isBarDown = !isBarDown;
        }

        // Fine adjustment (+/- 0.01) via D-Pad Up / Down
        if (dpadUpPressed) {
            offset = Math.min(0.5, offset + 0.01);
        }
        if (dpadDownPressed) {
            offset = Math.max(0.0, offset - 0.01);
        }

        // Coarse adjustment (+/- 0.05) via D-Pad Right / Left
        if (dpadRightPressed) {
            offset = Math.min(0.5, offset + 0.05);
        }
        if (dpadLeftPressed) {
            offset = Math.max(0.0, offset - 0.05);
        }

        // Apply positions to mirrored servos
        if (isBarDown) {
            // Down Position: Right servo gets +offset, Left servo gets -offset
            barRight.setPosition(Constants.Servos.BAR_NEUTRAL_POS + offset);
            barLeft.setPosition(Constants.Servos.BAR_NEUTRAL_POS - offset);
        } else {
            // Neutral Up Position: Both servos at 0.5
            barLeft.setPosition(Constants.Servos.BAR_NEUTRAL_POS);
            barRight.setPosition(Constants.Servos.BAR_NEUTRAL_POS);
        }

        previousY = gamepad1.y;
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;

        // Telemetry Display
        telemetry.addData("=== FLOWER BAR CALIBRATION ===", "");
        telemetry.addData("Bar Position", isBarDown ? "DOWN (Lowered)" : "UP (Neutral 0.5)");
        telemetry.addData("RECOMMENDED BAR_OFFSET", String.format(Locale.US, "%.3f", offset));
        telemetry.addData("barLeft Servo Position", String.format(Locale.US, "%.3f (0.5 - %.3f)", barLeft.getPosition(), offset));
        telemetry.addData("barRight Servo Position", String.format(Locale.US, "%.3f (0.5 + %.3f)", barRight.getPosition(), offset));
        telemetry.addLine("");
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Y Button         : Toggle Bar UP <-> DOWN");
        telemetry.addLine("D-Pad Up / Down  : Fine Adjust Offset (+/- 0.01)");
        telemetry.addLine("D-Pad Right/Left : Coarse Adjust Offset (+/- 0.05)");
        telemetry.update();
    }
}
