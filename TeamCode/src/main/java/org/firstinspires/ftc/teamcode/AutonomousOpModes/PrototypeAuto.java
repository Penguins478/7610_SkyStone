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


@Autonomous(name = "PrototypeAuto", group = "Autonomous")
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
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_INCHES = 3; // or 3 based on what we get
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
            encoderDrive(0.65, 24, 1000, 20);
            stop();
            //sleep(10000);
        }

    }

    public void encoderDrive(double power, double distance, double timeOuts, double error){
        double target = tl_motor.getCurrentPosition() + distance*COUNTS_PER_INCH*2;
        boolean checktl=true;
        boolean checktr=true;
        boolean checkbl=true;
        boolean checkbr=true;
        while(tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())>=error &&
                tr_motor.getCurrentPosition()<=target&&Math.abs(target-tr_motor.getCurrentPosition())>=error &&
                bl_motor.getCurrentPosition()<=target&&Math.abs(target-bl_motor.getCurrentPosition())>=error &&
                br_motor.getCurrentPosition()<=target&&Math.abs(target-br_motor.getCurrentPosition())>=error &&
                checkbl&&checkbr&&checktl&&checktr){

            telemetry.addData("lmao", "bruh");
            telemetry.update();

            if(checktl&&tl_motor.getCurrentPosition()<=target&&Math.abs(target-tl_motor.getCurrentPosition())>=error){
                tl_motor.setPower(power*((target-tl_motor.getCurrentPosition())/target));
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
}

