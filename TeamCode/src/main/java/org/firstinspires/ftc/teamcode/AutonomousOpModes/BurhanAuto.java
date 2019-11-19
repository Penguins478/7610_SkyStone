package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Burhan Auto", group = "Autonomous")
public class BurhanAuto extends LinearOpMode {
    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private BNO055IMU imu;
    private BNO055IMU.Parameters params;
    private double angle;

    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double GEAR_RATIO = 2;
    private static final double WHEEL_DIAMETER_INCHES = 2.9527559055; // or 3 based on what we get
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);
    private static final double ANGLE_ERROR = 5;

    public void runOpMode() {

        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        tl_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bl_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        tr_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        br_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        tl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        waitForStart();

        while (opModeIsActive()) {

            encoderDrive2(48, 'y', 1, 50, 500);
            encoderDrive2(-24, 'x', 1, 50, 500);
            encoderDrive2(-48, 'y', 1, 50, 500);
            encoderDrive2(24, 'x', 1, 50, 500);

            telemetry.update();

            break;
        }

        stop();
    }

    private void encoderDrive2(double inches, char direction, double power, double error, long timeout) {
        double dist = inches * COUNTS_PER_INCH;

        double tl_dist = 0;
        double tr_dist = 0;
        double bl_dist = 0;
        double br_dist = 0;

        boolean there_tl = false;
        boolean there_tr = false;
        boolean there_bl = false;
        boolean there_br = false;

        double start_tl = tl_motor.getCurrentPosition();
        double start_tr = tr_motor.getCurrentPosition();
        double start_bl = bl_motor.getCurrentPosition();
        double start_br = br_motor.getCurrentPosition();

        if (direction == 'y') {
            tl_dist = dist;
            tr_dist = dist;
            bl_dist = dist;
            br_dist = dist;
        } else if (direction == 'x') {
            tl_dist = dist;
            tr_dist = -dist;
            bl_dist = -dist;
            br_dist = dist;
        }

        while (!(there_tl && there_tr && there_bl && there_br)) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            there_tl = adjust_motor(tl_motor, tl_dist, start_tl, power, error);
            there_tr = adjust_motor(tr_motor, tr_dist, start_tr, power, error);
            there_bl = adjust_motor(bl_motor, bl_dist, start_bl, power, error);
            there_br = adjust_motor(br_motor, br_dist, start_br, power, error);

            while (Math.abs(angle) > ANGLE_ERROR) {
                double adjustment = Math.abs(angle - ANGLE_ERROR) / 45;
                if (adjustment < 0.1) {
                    adjustment = 0.1;
                }
                if (angle > 0) {
                    tl_motor.setPower(adjustment);
                    tr_motor.setPower(-adjustment);
                    bl_motor.setPower(adjustment);
                    br_motor.setPower(-adjustment);
                } else {
                    tl_motor.setPower(-adjustment);
                    tr_motor.setPower(adjustment);
                    bl_motor.setPower(-adjustment);
                    br_motor.setPower(adjustment);
                }
                angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                telemetry.addData("angle", angle);
                telemetry.update();
            }

            telemetry.addData("angle", angle);
            telemetry.addData("tl position", tl_motor.getCurrentPosition());
            telemetry.addData("tl goal", start_tl + tl_dist);
            telemetry.addData("tr position", tr_motor.getCurrentPosition());
            telemetry.addData("tr goal", start_tr + tr_dist);
            telemetry.addData("bl position", bl_motor.getCurrentPosition());
            telemetry.addData("bl goal", start_bl + bl_dist);
            telemetry.addData("br position", br_motor.getCurrentPosition());
            telemetry.addData("br goal", start_br + br_dist);

            telemetry.update();
        }

        sleep(timeout);
    }

    private boolean adjust_motor(DcMotor motor, double distance, double start, double power, double error) {
        double coeff = (start + distance - motor.getCurrentPosition()) / distance;
        if (coeff < 0.25) {
            coeff = 0.25;
        }
        if (motor.getCurrentPosition() < start + distance - error) {
            motor.setPower(coeff * power);
            return false;
        } else if (motor.getCurrentPosition() > start + distance + error) {
            motor.setPower(-coeff * power);
            return false;
        } else {
            motor.setPower(0);
            return true;
        }
    }

}
