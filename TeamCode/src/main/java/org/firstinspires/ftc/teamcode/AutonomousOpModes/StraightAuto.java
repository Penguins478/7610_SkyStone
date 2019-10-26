package org.firstinspires.ftc.teamcode.AutonomousOpModes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class StraightAuto extends LinearOpMode {
    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private static final double COUNTS_PER_MOTOR_REV = 288;
    private static final double GEAR_RATIO = 2;
    private static final double WHEEL_DIAMETER_INCHES = 3;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);

    private static final double K = 45;

    @Override
    public void runOpMode() {
        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        tl_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        bl_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        tr_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        tl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {

        }
    }

    private void straightDrive(String direction, double inches) {
        double x;
        double y;
        double r;

        if (direction == "up") {
            y = 1;
        }
        if (direction == "down") {
            y = -1;
        }
        if (direction == "right") {
            x = 1;
        }
        if (direction == "left") {
            x = -1;
        }

        tl_motor.setTargetPosition((int) (tl_motor.getCurrentPosition() + inches * COUNTS_PER_INCH));
        tr_motor.setTargetPosition((int) (tr_motor.getCurrentPosition() + inches * COUNTS_PER_INCH));
        bl_motor.setTargetPosition((int) (bl_motor.getCurrentPosition() + inches * COUNTS_PER_INCH));
        br_motor.setTargetPosition((int) (br_motor.getCurrentPosition() + inches * COUNTS_PER_INCH));


    }
}
