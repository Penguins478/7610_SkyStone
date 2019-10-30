
package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "TestAuto", group = "Autonomous")
//@Disabled
public class TestAuto extends LinearOpMode {           // hard code for now cuz we arent doing anything
    // and roadrunner + odometry will take awhile
    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    BNO055IMU imu;

    private double P_TURN_COEFF = 0.01;
    private double HEADING_THRESHOLD = 1;

    private BNO055IMU.Parameters parameters;
    Orientation angles;

    private static final double COUNTS_PER_MOTOR_REV = 288 * 4;
    private static final double GEAR_RATIO = 2;
    private static final double WHEEL_DIAMETER_INCHES = 2.9527559055; // or 3 based on what we get
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

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


        while(opModeIsActive()){

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            encoderDrive(48 * COUNTS_PER_INCH, 48 * COUNTS_PER_INCH, 48 * COUNTS_PER_INCH, 48 * COUNTS_PER_INCH, 0.65, 3, 100);
            sleep(500);
//        encoderDrive(24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.3, 0.5, 100);
//        sleep(500);

            encoderDrive(-24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, 0.65, 3, 100);
            sleep(500);

            encoderDrive(-24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, 0.65, 3, 100);
            sleep(500);

            encoderDrive(0, -24 * COUNTS_PER_INCH * Math.sqrt(2), -24 * COUNTS_PER_INCH * Math.sqrt(2), 0, 0.85, 3, 100);
            sleep(500);

            //encoderDrive(24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.3, 0.5, 100);
            //sleep(500);
            break;
        }
        stop();
    }

    public void encoderDrive(double distance1, double distance2, double distance3, double distance4, double power, double error, long timeout){

        double start_tl = tl_motor.getCurrentPosition();
        double start_tr = tr_motor.getCurrentPosition();
        double start_bl = bl_motor.getCurrentPosition();
        double start_br = br_motor.getCurrentPosition();

        boolean there_tl = false;
        boolean there_tr = false;
        boolean there_bl = false;
        boolean there_br = false;

        while(!(there_tl && there_tr && there_bl && there_br)) {
            if(!there_tl) {
                if (tl_motor.getCurrentPosition() < (start_tl + distance1 - error) / 2) {
                    tl_motor.setPower(power);
                } else if (tl_motor.getCurrentPosition() >= (start_tl + distance1 - error) / 2 && tl_motor.getCurrentPosition() < start_tl + distance1 - error) {
                    tl_motor.setPower(power * 0.5);
                } else if (tl_motor.getCurrentPosition() > start_tl + distance1 + error) {
                    //tl_motor.setPower(power * -0.5);
                    tl_motor.setPower(-power * 0.5);
                } else {
                    there_tl = true;
                    tl_motor.setPower(0);
                }
            }
            if(!there_tr) {
                if (tr_motor.getCurrentPosition() < (start_tr + distance2 - error) / 2) {
                    tr_motor.setPower(power);
                } else if (tr_motor.getCurrentPosition() >= (start_tr + distance2 - error) / 2 && tr_motor.getCurrentPosition() < start_tr + distance2 - error) {
                    tr_motor.setPower(power * 0.5);
                } else if (tr_motor.getCurrentPosition() > start_tr + distance2 + error) {
                    //tr_motor.setPower(power * -0.5);
                    tr_motor.setPower(-power * 0.5);
                } else {
                    there_tr = true;
                    tr_motor.setPower(0);
                }
            }
            if(!there_bl) {
                if (bl_motor.getCurrentPosition() < (start_bl + distance3 - error) / 2) {
                    bl_motor.setPower(power);
                } else if (bl_motor.getCurrentPosition() >= (start_bl + distance3 - error) / 2 && bl_motor.getCurrentPosition() < start_bl + distance3 - error) {
                    bl_motor.setPower(power * 0.5);
                } else if (bl_motor.getCurrentPosition() > start_bl + distance3 + error) {
                    //bl_motor.setPower(power * -0.5);
                    bl_motor.setPower(-power * 0.5);
                } else {
                    there_bl = true;
                    bl_motor.setPower(0);
                }
            }
            if(!there_br) {
                if (br_motor.getCurrentPosition() < (start_br + distance4 - error) / 2) {
                    br_motor.setPower(power);
                } else if (br_motor.getCurrentPosition() >= (start_br + distance4 - error) / 2 && br_motor.getCurrentPosition() < start_br + distance4 - error) {
                    br_motor.setPower(power * 0.5);
                } else if (br_motor.getCurrentPosition() > start_br + distance4 + error) {
                    //br_motor.setPower(power * -0.5);
                    br_motor.setPower(-power * 0.5);
                } else {
                    there_br = true;
                    br_motor.setPower(0);
                }
            }

            telemetry.addData("tl power", tl_motor.getPower());
            telemetry.addData("tr power", tr_motor.getPower());
            telemetry.addData("bl power", bl_motor.getPower());
            telemetry.addData("br power", br_motor.getPower());
            telemetry.addData("Encoder tl", tl_motor.getCurrentPosition());
            telemetry.addData("Encoder tr", tr_motor.getCurrentPosition());
            telemetry.addData("Encoder bl", bl_motor.getCurrentPosition());
            telemetry.addData("Encoder br", br_motor.getCurrentPosition());
            telemetry.addData("Encoder tl goal", start_tl + distance1);
            telemetry.addData("Encoder tr goal", start_tr + distance2);
            telemetry.addData("Encoder bl goal", start_bl + distance3);
            telemetry.addData("Encoder br goal", start_br + distance4);
            telemetry.update();
        }

        sleep(timeout);

    }
//
//    public void gyroTurn (  double speed, double angle) {
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double initialPosition = angles.firstAngle;
//
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, initialPosition)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//        }
//
//    }
//
//    boolean onHeading(double speed, double angle, double errorMult, double initialPosition) {
//        double   error ;
//        double   steer ;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//
//        // determine turn power based on +/- error
//        error = getError(angle, initialPosition);
//
//        if (Math.abs(error) <= HEADING_THRESHOLD) {
//            steer = 0.0;
//            leftSpeed  = 0.0;
//            rightSpeed = 0.0;
//            onTarget = true;
//        }
//        else {
//            steer = getSteer(error, errorMult);
//            rightSpeed  = speed * steer;
//            leftSpeed   = -rightSpeed;
//        }
//
//        // Send desired speeds to motors.
//
//        tl_motor.setPower(leftSpeed);
//        tr_motor.setPower(rightSpeed);
//        bl_motor.setPower(leftSpeed);
//        br_motor.setPower(rightSpeed);
//
//
//        // Display it for the driver.
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//
//        return onTarget;
//    }
//
//
//    public double getError(double targetAngle, double initialPosition) {
//
//        double robotError;
//
//        // calculate error in -179 to +180 range  (
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        robotError = targetAngle - angles.firstAngle + initialPosition;
//        while (robotError > 180)  robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }
//
//    public double getSteer(double error, double errorMult) {
//        if(error * errorMult < 0) {
//            return Range.clip(error * errorMult, -0.5, -0.01);
//        }
//        else {
//            return Range.clip(error * errorMult, 0.01, 0.5);
//        }
//    }
}