package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PrototypeAuto", group = "Autonomous")
@Disabled
public class PrototypeAuto extends LinearOpMode {           // hard code for now cuz we arent doing anything
    // and roadrunner + odometry will take awhile
    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private static final double COUNTS_PER_MOTOR_REV = 288;
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_INCHES = 4; // or 3 based on what we get
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){

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

        while(opModeIsActive()) {   // go implement straight drive

            // go up to the stones
            tl_motor.setTargetPosition((int) (tl_motor.getCurrentPosition() + (47 - 5) * COUNTS_PER_INCH));
            tr_motor.setTargetPosition((int) (tr_motor.getCurrentPosition() + (47 - 5) * COUNTS_PER_INCH));
            bl_motor.setTargetPosition((int) (bl_motor.getCurrentPosition() + (47 - 5) * COUNTS_PER_INCH));
            br_motor.setTargetPosition((int) (br_motor.getCurrentPosition() + (47 - 5) * COUNTS_PER_INCH));

            // can lower speed if needed
            tl_motor.setPower(1);
            tr_motor.setPower(1);
            bl_motor.setPower(1);
            br_motor.setPower(1);


            // get near the wall to have vision on all stones
            tl_motor.setTargetPosition((int) (tl_motor.getCurrentPosition() + (24 - 4) * COUNTS_PER_INCH));
            tr_motor.setTargetPosition((int) (tr_motor.getCurrentPosition() - (24 - 4) * COUNTS_PER_INCH));
            bl_motor.setTargetPosition((int) (bl_motor.getCurrentPosition() - (24 - 4) * COUNTS_PER_INCH));
            br_motor.setTargetPosition((int) (br_motor.getCurrentPosition() + (24 - 4) * COUNTS_PER_INCH));

            // can lower speed if needed
            tl_motor.setPower(1);
            tr_motor.setPower(1);
            bl_motor.setPower(1);
            br_motor.setPower(1);

            tl_motor.setTargetPosition((int) (tl_motor.getCurrentPosition() - (48 - 0) * COUNTS_PER_INCH));
            tr_motor.setTargetPosition((int) (tr_motor.getCurrentPosition() + (48 - 0) * COUNTS_PER_INCH));
            bl_motor.setTargetPosition((int) (bl_motor.getCurrentPosition() + (48 - 0) * COUNTS_PER_INCH));
            br_motor.setTargetPosition((int) (br_motor.getCurrentPosition() - (48 - 0) * COUNTS_PER_INCH));

            // can lower speed if needed
            tl_motor.setPower(1);
            tr_motor.setPower(1);
            bl_motor.setPower(1);
            br_motor.setPower(1);

            // diagonal towards the wall
            //tl_motor.setTargetPosition(0);
            tr_motor.setTargetPosition((int) (tr_motor.getCurrentPosition() + (48 - 0) * COUNTS_PER_INCH));
            //bl_motor.setTargetPosition(0);
            br_motor.setTargetPosition((int) (br_motor.getCurrentPosition() - (48 - 0) * COUNTS_PER_INCH));

            // can lower speed if needed
            tl_motor.setPower(0);
            tr_motor.setPower(1);
            bl_motor.setPower(0);
            br_motor.setPower(1);

            telemetry.addData("Encoder tl", tl_motor.getCurrentPosition());
            telemetry.addData("Encoder tr", tr_motor.getCurrentPosition());
            telemetry.addData("Encoder bl", bl_motor.getCurrentPosition());
            telemetry.addData("Encoder br", br_motor.getCurrentPosition());
            telemetry.update();
        }
    }
}

