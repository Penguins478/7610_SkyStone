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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "VuforiaAuto", group = "Autonomous")
@Disabled
public class VuforiaAuto extends LinearOpMode {           // hard code for now cuz we arent doing anything
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

    // vu

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;


    private static final String VUFORIA_KEY =
            " AbBJWbv/////AAABmVD+Kn/RREico/taojg6bfsKOtum5LJz26FaanI9FmsViq9h3H32Cdrvha/AEzmlNBaQoOc5vu047HjDtjOSkV6b6W4sQ/YupBfJOCKaVpy1OYJPHdcMEucd20jEvRCT+3ECY2oEpgTXsU1DJMd47eehl/ueiA+V/EdvC2MuVBdyb+B0CLlSGjfGt63iAT9Orm4j42bCM1tmtsZGtJO/bWqLax8iMRctVK7I0mZpQ2B6/v+EV3mPN8si+ezfh3gtwyjBTQWUhYDMy4yXyDjoBhlPVzTnThJ6OpXH3EqQv0ODVLiizoX0ddq9QU2hQINKTJI1M0VWfRlvpxe9q+F0oAwP7f6GQOKC5MhsCquUpvqM ";


    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor


    private static final float stoneZ = 2.00f * mmPerInch;


    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;


    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;


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

        //

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


        targetsSkyStone.activate();
        //

        waitForStart();


        while(opModeIsActive()){

            encoderDrive(20 * COUNTS_PER_INCH, 20 * COUNTS_PER_INCH, 20 * COUNTS_PER_INCH, 20 * COUNTS_PER_INCH, 0.65, 10, 100);
            sleep(500);

            double prev_tl = tl_motor.getCurrentPosition();
            double prev_tr = tr_motor.getCurrentPosition();
            double prev_bl = bl_motor.getCurrentPosition();
            double prev_br = br_motor.getCurrentPosition();

            double cur_tl = prev_tl;
            double cur_tr = prev_tr;
            double cur_bl = prev_bl;
            double cur_br = prev_br;

            while(!targetVisible) {
                targetVisible = false;
                //
                //set motor powers
                tl_motor.setPower(0.25);
                tr_motor.setPower(-0.25);
                bl_motor.setPower(-0.25);
                br_motor.setPower(-0.25);

                cur_tl = tl_motor.getCurrentPosition();
                cur_tr = tr_motor.getCurrentPosition();
                cur_bl = bl_motor.getCurrentPosition();
                cur_br = br_motor.getCurrentPosition();

                //
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }
            }

            if (targetVisible) {
                //
                //use mech
                //
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
            //drive
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double driveBridge_tl = 72 * COUNTS_PER_INCH;
            double driveBridge_tr = -72 * COUNTS_PER_INCH;
            double driveBridge_bl = -72 * COUNTS_PER_INCH;
            double driveBridge_br = 72 * COUNTS_PER_INCH;


            encoderDrive(driveBridge_tl - cur_tl, (driveBridge_tr - cur_tr) * -1, (driveBridge_bl - cur_bl) * -1, driveBridge_br - cur_br, 0.65, 10, 100);

            // drop

            encoderDrive(-24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, 0.65, 10, 100);
            sleep(500);

            encoderDrive(-driveBridge_tl, driveBridge_tr, driveBridge_bl, -driveBridge_br, 0.85, 10, 100);
            sleep(500);


            encoderDrive(24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.65, 10, 100);
            sleep(500);
            //encoderDrive(24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.3, 0.5, 100);
            //sleep(500);
            while(!targetVisible) {
                targetVisible = false;
                //
                //set motor powers
                tl_motor.setPower(0.25);
                tr_motor.setPower(-0.25);
                bl_motor.setPower(-0.25);
                br_motor.setPower(-0.25);

                cur_tl = tl_motor.getCurrentPosition();
                cur_tr = tr_motor.getCurrentPosition();
                cur_bl = bl_motor.getCurrentPosition();
                cur_br = br_motor.getCurrentPosition();

                //
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }
            }

            if (targetVisible) {
                //
                //use mech
                //
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
            encoderDrive(driveBridge_tl - cur_tl, (driveBridge_tr - cur_tr) * -1, (driveBridge_bl - cur_bl) * -1, driveBridge_br - cur_br, 0.65, 10, 100);
//drop rock
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
                    tl_motor.setPower(power * 0.75);
                } else if (tl_motor.getCurrentPosition() > start_tl + distance1 + error) {
                    //tl_motor.setPower(power * -0.5);
                    tl_motor.setPower(-power * 0.75);
                } else {
                    there_tl = true;
                    tl_motor.setPower(0);
                }
            }
            if(!there_tr) {
                if (tr_motor.getCurrentPosition() < (start_tr + distance2 - error) / 2) {
                    tr_motor.setPower(power);
                } else if (tr_motor.getCurrentPosition() >= (start_tr + distance2 - error) / 2 && tr_motor.getCurrentPosition() < start_tr + distance2 - error) {
                    tr_motor.setPower(power * 0.75);
                } else if (tr_motor.getCurrentPosition() > start_tr + distance2 + error) {
                    //tr_motor.setPower(power * -0.5);
                    tr_motor.setPower(-power * 0.75);
                } else {
                    there_tr = true;
                    tr_motor.setPower(0);
                }
            }
            if(!there_bl) {
                if (bl_motor.getCurrentPosition() < (start_bl + distance3 - error) / 2) {
                    bl_motor.setPower(power);
                } else if (bl_motor.getCurrentPosition() >= (start_bl + distance3 - error) / 2 && bl_motor.getCurrentPosition() < start_bl + distance3 - error) {
                    bl_motor.setPower(power * 0.75);
                } else if (bl_motor.getCurrentPosition() > start_bl + distance3 + error) {
                    //bl_motor.setPower(power * -0.5);
                    bl_motor.setPower(-power * 0.75);
                } else {
                    there_bl = true;
                    bl_motor.setPower(0);
                }
            }
            if(!there_br) {
                if (br_motor.getCurrentPosition() < (start_br + distance4 - error) / 2) {
                    br_motor.setPower(power);
                } else if (br_motor.getCurrentPosition() >= (start_br + distance4 - error) / 2 && br_motor.getCurrentPosition() < start_br + distance4 - error) {
                    br_motor.setPower(power * 0.75);
                } else if (br_motor.getCurrentPosition() > start_br + distance4 + error) {
                    //br_motor.setPower(power * -0.5);
                    br_motor.setPower(-power * 0.75);
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




