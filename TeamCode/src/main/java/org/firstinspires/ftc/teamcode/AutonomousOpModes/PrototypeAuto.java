package org.firstinspires.ftc.teamcode.AutonomousOpModes;


import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;




@Autonomous(name = "PrototypeAuto", group = "Autonomous") //broken

//@Disabled
public class PrototypeAuto extends LinearOpMode {           // hard code for now cuz we arent doing anything
    // and roadrunner + odometry will take awhile
    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;
    private BNO055IMU imu;
    private Orientation angles;


    private static final double COUNTS_PER_MOTOR_REV = 288;
    private static final double GEAR_RATIO = 2;
    private static final double WHEEL_DIAMETER_INCHES = 2.9527559055; // or 3 based on what we get
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);



    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Set up our telemetry dashboard
        composeTelemetry();

        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");


        tl_motor.setDirection(DcMotor.Direction.FORWARD);
        bl_motor.setDirection(DcMotor.Direction.FORWARD);
        tr_motor.setDirection(DcMotor.Direction.REVERSE);
        br_motor.setDirection(DcMotor.Direction.REVERSE);

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

        //while(opModeIsActive()) {   // go implement straight drive
        forward(0.65, 24, 1000, 20);
        //right(0.6, 24, 1000, 25);
        stop();
        //sleep(10000);
        //}


    }

    public void forward(double power, double distance, double timeOuts, double error){
        double target = tl_motor.getCurrentPosition() + distance*COUNTS_PER_INCH*4;
        boolean checktl=true;
        boolean checktr=true;
        boolean checkbl=true;
        boolean checkbr=true;
        while(tl_motor.getCurrentPosition() <= target && Math.abs(target-tl_motor.getCurrentPosition())>=error &&
                tr_motor.getCurrentPosition() <= target && Math.abs(target-tr_motor.getCurrentPosition())>=error &&
                bl_motor.getCurrentPosition() <= target && Math.abs(target-bl_motor.getCurrentPosition())>=error &&
                br_motor.getCurrentPosition() <= target && Math.abs(target-br_motor.getCurrentPosition())>=error &&
                checkbl&&checkbr&&checktl&&checktr){

            if(checktl&&tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())>=error){
                tl_motor.setPower(power*Math.abs((target-tl_motor.getCurrentPosition())/target));
            }else if(checktl&&tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())<=error){
                checktl=false;
                tl_motor.setPower(0);
            }
            if(checktr&&tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())>=error){
                tr_motor.setPower(power*((target-tr_motor.getCurrentPosition())/target));
            }else if(checktr&&tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())<=error){
                checktr=false;
                tr_motor.setPower(0);
            }
            if(checkbl&&bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())>=error){
                bl_motor.setPower(power*((target-bl_motor.getCurrentPosition())/target));
            }else if(checkbl&&bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())<=error){
                checkbl=false;
                bl_motor.setPower(0);
            }
            if(checkbr&&br_motor.getCurrentPosition()<=target&&Math.abs(target-br_motor.getCurrentPosition())>=error){
                br_motor.setPower(power*((target-br_motor.getCurrentPosition())/target));
            }else if(checkbr&&br_motor.getCurrentPosition()<=target&&Math.abs(target-br_motor.getCurrentPosition())<=error){
                checkbr=false;
                br_motor.setPower(0);

            }


        }
        sleep((long)timeOuts);
    }

    public void backward(double power, double distance, double timeOuts, double error){
        double target = tl_motor.getCurrentPosition() + distance*COUNTS_PER_INCH*4;
        boolean checktl=true;
        boolean checktr=true;
        boolean checkbl=true;
        boolean checkbr=true;
        while(tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())>=error &&
                tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())>=error &&
                bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())>=error &&
                br_motor.getCurrentPosition()<=target&&Math.abs(target-br_motor.getCurrentPosition())>=error &&
                checkbl&&checkbr&&checktl&&checktr){

            if(checktl&&tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())>=error){
                tl_motor.setPower(-power*Math.abs((target-tl_motor.getCurrentPosition())/target));
            }else if(checktl&&tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())<=error){
                checktl=false;
                tl_motor.setPower(0);
            }
            if(checktr&&tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())>=error){
                tr_motor.setPower(-power*((target-tr_motor.getCurrentPosition())/target));
            }else if(checktr&&tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())<=error){
                checktr=false;
                tr_motor.setPower(0);
            }
            if(checkbl&&bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())>=error){
                bl_motor.setPower(-power*((target-bl_motor.getCurrentPosition())/target));
            }else if(checkbl&&bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())<=error){
                checkbl=false;
                bl_motor.setPower(0);
            }
            if(checkbr&&br_motor.getCurrentPosition()<=target&&Math.abs(target-br_motor.getCurrentPosition())>=error){
                br_motor.setPower(-power*((target-br_motor.getCurrentPosition())/target));
            }else if(checkbr&&br_motor.getCurrentPosition()<=target&&Math.abs(target-br_motor.getCurrentPosition())<=error){
                checkbr=false;
                br_motor.setPower(0);

            }


        }
        sleep((long)timeOuts);
    }

    public void left(double power, double distance, double timeOuts, double error){
        double target = tl_motor.getCurrentPosition() + distance*COUNTS_PER_INCH*4;
        boolean checktl=true;
        boolean checktr=true;
        boolean checkbl=true;
        boolean checkbr=true;
        while(tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())>=error &&
                tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())>=error &&
                bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())>=error &&
                br_motor.getCurrentPosition()<=target&&Math.abs(target-br_motor.getCurrentPosition())>=error &&
                checkbl&&checkbr&&checktl&&checktr){

            if(checktl&&tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())>=error){
                tl_motor.setPower(-power*Math.abs((target-tl_motor.getCurrentPosition())/target));
            }else if(checktl&&tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())<=error){
                checktl=false;
                tl_motor.setPower(0);
            }
            if(checktr&&tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())>=error){
                tr_motor.setPower(power*Math.abs((target-tr_motor.getCurrentPosition())/target));
            }else if(checktr&&tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())<=error){
                checktr=false;
                tr_motor.setPower(0);
            }
            if(checkbl&&bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())>=error){
                bl_motor.setPower(power*Math.abs((target-bl_motor.getCurrentPosition())/target));
            }else if(checkbl&&bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())<=error){
                checkbl=false;
                bl_motor.setPower(0);
            }
            if(checkbr&&br_motor.getCurrentPosition()<=target&&Math.abs(target-br_motor.getCurrentPosition())>=error){
                br_motor.setPower(-power*Math.abs((target-br_motor.getCurrentPosition())/target));
            }else if(checkbr&&br_motor.getCurrentPosition()<=target&&Math.abs(target-br_motor.getCurrentPosition())<=error){
                checkbr=false;
                br_motor.setPower(0);

            }


        }
        sleep((long)timeOuts);
    }

    public void right(double power, double distance, double timeOuts, double error){
        double target = tl_motor.getCurrentPosition() + distance*COUNTS_PER_INCH*4;
        boolean checktl=true;
        boolean checktr=true;
        boolean checkbl=true;
        boolean checkbr=true;
        while(tl_motor.getCurrentPosition()>=-target&&Math.abs(target-tl_motor.getCurrentPosition())>=error &&
                tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())>=error &&
                bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())>=error &&
                br_motor.getCurrentPosition()>=-target&&Math.abs(target-br_motor.getCurrentPosition())>=error &&
                checkbl&&checkbr&&checktl&&checktr){

            if(checktl&&tl_motor.getCurrentPosition()>=-target&&Math.abs(target-tl_motor.getCurrentPosition())>=error){
                tl_motor.setPower(-power*Math.abs((target-tl_motor.getCurrentPosition()))/target);
            }else if(checktl&&Math.abs(target-tl_motor.getCurrentPosition())<=error){
                checktl=false;
                tl_motor.setPower(0);
            }
            if(checktr&&tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())>=error){
                tr_motor.setPower(power*Math.abs((target-tr_motor.getCurrentPosition()))/target);
            }else if(checktr&&Math.abs(target-tr_motor.getCurrentPosition())<=error){
                checktr=false;
                tr_motor.setPower(0);
            }
            if(checkbl&&bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())>=error){
                bl_motor.setPower(power*Math.abs((target-bl_motor.getCurrentPosition()))/target);
            }else if(checkbl&&Math.abs(target-bl_motor.getCurrentPosition())<=error){
                checkbl=false;
                bl_motor.setPower(0);
            }
            if(checkbr&&br_motor.getCurrentPosition()>=-target&&Math.abs(target-br_motor.getCurrentPosition())>=error){
                br_motor.setPower(-power*Math.abs((target-br_motor.getCurrentPosition()))/target);
            }else if(checkbr&&Math.abs(target-br_motor.getCurrentPosition())<=error){
                checkbr=false;
                br_motor.setPower(0);

            }


        }
        sleep((long)timeOuts);
    }

    public void gyroTurn(double degrees){
        double cur = angles.firstAngle;

        while(degrees<cur){

        }
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });


    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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


