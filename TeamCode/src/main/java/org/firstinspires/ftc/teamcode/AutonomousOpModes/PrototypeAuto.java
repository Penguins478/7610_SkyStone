package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PrototypeAuto", group = "Autonomous") //broken
//@Disabled
public class PrototypeAuto extends LinearOpMode {           // hard code for now cuz we arent doing anything
    // and roadrunner + odometry will take awhile
    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private static final double COUNTS_PER_MOTOR_REV = 288;
    private static final double GEAR_RATIO = 2;
    private static final double WHEEL_DIAMETER_INCHES = 2.9527559055; // or 3 based on what we get
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){

        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        tl_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bl_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        tr_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        br_motor.setDirection(DcMotorSimple.Direction.FORWARD);

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

        waitForStart();

        while(opModeIsActive()) {   // go implement straight drive

            encoderDrive(10 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0.3, 25, 100);


            telemetry.addData("Encoder tl", tl_motor.getCurrentPosition());
            telemetry.addData("Encoder tr", tr_motor.getCurrentPosition());
            telemetry.addData("Encoder bl", bl_motor.getCurrentPosition());
            telemetry.addData("Encoder br", br_motor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void encoderDrive(double distance1, double distance2, double distance3, double distance4, double power, double error, long timeout){
        //if grater, -power
        // if less ,power
        // inside margin ,0
        double start_tl = tl_motor.getCurrentPosition();
        double start_tr = tr_motor.getCurrentPosition();
        double start_bl = bl_motor.getCurrentPosition();
        double start_br = br_motor.getCurrentPosition();

        while(Math.abs(tl_motor.getCurrentPosition() - (tl_motor.getCurrentPosition() + distance1)) > 10
                || Math.abs(tr_motor.getCurrentPosition() - (tr_motor.getCurrentPosition() + distance2)) > 10
                || Math.abs(bl_motor.getCurrentPosition() - (bl_motor.getCurrentPosition() + distance3)) > 10
                || Math.abs(br_motor.getCurrentPosition() - (br_motor.getCurrentPosition() + distance4)) > 10){
            if(tl_motor.getCurrentPosition() + error < start_tl + distance1){
                tl_motor.setPower(power);
            }else if(tl_motor.getCurrentPosition() > start_tl + distance1 + error){
                tl_motor.setPower(power * 0.5);
            }else{
                tl_motor.setPower(0);
            }
            if(tr_motor.getCurrentPosition() + error < start_tr + distance2){
                tr_motor.setPower(power);
            }else if(tr_motor.getCurrentPosition() > start_tr + distance2 + error){
                tr_motor.setPower(power * 0.5);
            }else{
                tr_motor.setPower(0);
            }
            if(bl_motor.getCurrentPosition() + error < start_bl + distance3){
                bl_motor.setPower(power);
            }else if(bl_motor.getCurrentPosition() > start_bl + distance3 + error){
                bl_motor.setPower(power * 0.5);
            }else{
                bl_motor.setPower(0);
            }
            if(br_motor.getCurrentPosition() + error < start_br + distance4){
                tl_motor.setPower(power);
            }else if(br_motor.getCurrentPosition() > start_br + distance4 + error){
                br_motor.setPower(power * 0.5);
            }else{
                br_motor.setPower(0);
            }
            telemetry.addData("Encoder tl", tl_motor.getCurrentPosition());
            telemetry.addData("Encoder tr", tr_motor.getCurrentPosition());
            telemetry.addData("Encoder bl", bl_motor.getCurrentPosition());
            telemetry.addData("Encoder br", br_motor.getCurrentPosition());
            telemetry.update();
        }

        sleep(timeout);

    }

}


