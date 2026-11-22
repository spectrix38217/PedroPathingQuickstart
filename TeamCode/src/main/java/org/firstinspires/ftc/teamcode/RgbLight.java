package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * RgbLight - Driver for goBILDA RGB Indicator Light (3118-0808-0002) connected to a Servo Port.
 *
 * <p>Responds to standard Servo PWM signals (500µs to 2500µs mapped to 0.0 - 1.0 setPosition):
 * <ul>
 *   <li>OFF: 0.000 (500µs)</li>
 *   <li>RED: 0.300 (1100µs)</li>
 *   <li>ORANGE: 0.350 (1200µs)</li>
 *   <li>YELLOW: 0.400 (1300µs)</li>
 *   <li>SAGE: 0.450 (1400µs)</li>
 *   <li>GREEN: 0.500 (1500µs)</li>
 *   <li>AZURE / CYAN: 0.550 (1600µs)</li>
 *   <li>BLUE: 0.600 (1700µs)</li>
 *   <li>INDIGO: 0.650 (1800µs)</li>
 *   <li>VIOLET / PURPLE: 0.700 (1900µs)</li>
 *   <li>WHITE: 1.000 (2500µs)</li>
 * </ul>
 */
public class RgbLight {

    public enum Color {
        OFF(0.000),
        RED(0.300),
        ORANGE(0.350),
        YELLOW(0.400),
        SAGE(0.450),
        GREEN(0.500),
        AZURE(0.550),
        CYAN(0.550),
        BLUE(0.600),
        INDIGO(0.650),
        VIOLET(0.700),
        PURPLE(0.700),
        WHITE(1.000);

        private final double position;

        Color(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public static final String DEFAULT_DEVICE_NAME = "light";

    private final Servo lightServo;
    private Color currentColor = Color.PURPLE;
    private final ElapsedTime cycleTimer = new ElapsedTime();

    public RgbLight(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_DEVICE_NAME);
    }

    public RgbLight(HardwareMap hardwareMap, String deviceName) {
        lightServo = hardwareMap.get(Servo.class, deviceName);
        if (lightServo instanceof ServoImplEx) {
            ((ServoImplEx) lightServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        setPurple();
    }

    /**
     * Sets the light to a specific Color enum preset.
     */
    public void setColor(Color color) {
        this.currentColor = color;
        lightServo.setPosition(color.getPosition());
    }

    /**
     * Sets the light servo position directly (clamped 0.0 to 1.0).
     */
    public void setPosition(double position) {
        double clamped = Math.min(1.0, Math.max(0.0, position));
        lightServo.setPosition(clamped);
    }

    public Color getCurrentColor() {
        return currentColor;
    }

    public double getPosition() {
        return lightServo.getPosition();
    }

    // Convenience color methods
    public void setPurple() {
        setColor(Color.PURPLE);
    }

    public void setCyan() {
        setColor(Color.CYAN);
    }

    public void setOff() {
        setColor(Color.OFF);
    }

    public void setRed() {
        setColor(Color.RED);
    }

    public void setGreen() {
        setColor(Color.GREEN);
    }

    public void setBlue() {
        setColor(Color.BLUE);
    }

    public void setYellow() {
        setColor(Color.YELLOW);
    }

    public void setWhite() {
        setColor(Color.WHITE);
    }

    /**
     * Cycles between two colors with a specified time interval (in seconds).
     *
     * @param color1 First color.
     * @param color2 Second color.
     * @param intervalSeconds Duration of each phase in seconds (e.g. 0.5s).
     */
    public void cycle(Color color1, Color color2, double intervalSeconds) {
        if (intervalSeconds <= 0.0) return;
        double elapsed = cycleTimer.seconds();
        long step = (long) (elapsed / intervalSeconds);
        if (step % 2 == 0) {
            setColor(color1);
        } else {
            setColor(color2);
        }
    }

    /**
     * Cycles between Purple and Cyan at 0.5-second intervals.
     */
    public void cyclePurpleCyan() {
        cycle(Color.PURPLE, Color.CYAN, 0.5);
    }

    /**
     * Cycles between Purple and Cyan with a specified interval in seconds.
     */
    public void cyclePurpleCyan(double intervalSeconds) {
        cycle(Color.PURPLE, Color.CYAN, intervalSeconds);
    }

    /**
     * Resets the cycle timer.
     */
    public void resetCycleTimer() {
        cycleTimer.reset();
    }
}
