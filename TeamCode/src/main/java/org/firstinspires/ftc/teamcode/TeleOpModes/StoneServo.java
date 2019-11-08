package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "stone", group = "tleop")
//@Disabled
public class StoneServo extends LinearOpMode {


    private static final double MAXIMUM = 1; //change
    private static final double MINIMUM = 0; //change

    private Servo servo;

    public void runOpMode(){

        servo = hardwareMap.servo.get("stone_servo");

        servo.setPosition(MINIMUM);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                servo.setPosition(MAXIMUM);
            }else if(gamepad1.b){
                servo.setPosition(MINIMUM);
            }
            telemetry.addData("Servo Position: ", servo.getPosition());
            telemetry.update();
            idle();

        }
        stop();
    }

}
