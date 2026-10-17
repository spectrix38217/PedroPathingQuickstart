package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Blank OpMode", group = "Test")
public class TestBlankOpMode extends OpMode {
    @Override public void init() { telemetry.addLine("Chudbot Ready 3"); }
    @Override public void loop() {}
}