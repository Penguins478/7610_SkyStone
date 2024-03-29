package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servo", group = "teleop modes")
public class FoundationServo extends OpMode {
    private Servo servol;
    private Servo servor;

    @Override
    public void init() {
        servol = hardwareMap.servo.get("servol");
        servor = hardwareMap.servo.get("servor");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        boolean pressed = gamepad1.right_bumper;
        if (pressed) {
            servol.setPosition(1.0);
            servor.setPosition(1.0);
        } else {
            servol.setPosition(0.0);
            servor.setPosition(0.0);
        }
    }

    @Override
    public void stop() {

    }
}
