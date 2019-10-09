package org.firstinspires.ftc.teamcode.FirstYearTeleOps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Burhan_TeleOp", group = "Iterative OpMode")
public class Burhan_TeleOp extends OpMode {
    // declares all of the variables, like motors, and power
    private DcMotor tl;
    private DcMotor tr;
    private DcMotor bl;
    private DcMotor br;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation angles;
    private double tl_power;
    private double tr_power;
    private double bl_power;
    private double br_power;
    private double tl_prev_power;
    private double tr_prev_power;
    private double bl_prev_power;
    private double br_prev_power;
    private double x;
    private double y;
    private double r;
    private double angle;
    private final double k = 45;
    private boolean dpad_mode;
    private boolean up;
    private boolean down;
    private boolean left;
    private boolean right;
    private boolean change_mode;
    private double target_angle;
    private boolean accelerate;
    private boolean change_acceleration_mode;

    @Override
    public void init() {
        // initializes the motors, their directions and power, and the mode they are in
        // it also sets the powers
        tl = hardwareMap.dcMotor.get("tl");
        tr = hardwareMap.dcMotor.get("tr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        tl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        tr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        tl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tl_prev_power = 0;
        tr_prev_power = 0;
        bl_prev_power = 0;
        br_prev_power = 0;

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        dpad_mode = false;
        accelerate = true;
    }

    @Override
    public void init_loop() {
        //nothing happens
    }

    @Override
    public void loop() {
        // sets variables for controller stick
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        angle = angles.firstAngle;
        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;
        left = gamepad1.dpad_left;
        right = gamepad1.dpad_right;
        change_mode = gamepad1.right_bumper;
        change_acceleration_mode = gamepad1.left_bumper;

        if (change_mode) {
            dpad_mode = !dpad_mode;
            target_angle = angle;
        }

        if (change_acceleration_mode) {
            accelerate = !accelerate;
        }

        if (dpad_mode) {
            if (up) {
                y = 1;
            }
            if (down) {
                y = -1;
            }
            if (left) {
                x = -1;
            }
            if (right) {
                x = 1;
            }

            if (angle != target_angle) {
                r = (target_angle - angle) / k;
            }
        } else {
            x = -gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            r = -gamepad1.right_stick_x;

            double scalar = Math.hypot(x, y);
            double theta;

            if (angle != 0.0) {
                if (x == 0) {
                    if (y < 0) {
                        theta = -Math.PI / 2;
                    } else {
                        theta = Math.PI / 2;
                    }
                } else if (x < 0) {
                    theta = Math.atan(y / x) + Math.PI;
                } else {
                    theta = Math.atan(y / x);
                }
                theta -= angle;
                x = scalar * Math.cos(theta);
                y = scalar * Math.sin(theta);
            }
        }


        tl_power = y - x + r;
        tr_power = y + x - r;
        bl_power = y + x + r;
        br_power = y - x - r;

        tl_power = Range.clip(tl_power, -1, 1);
        tr_power = Range.clip(tr_power, -1, 1);
        bl_power = Range.clip(bl_power, -1, 1);
        br_power = Range.clip(br_power, -1, 1);

        if (accelerate) {
            tl_power = slow_accelerate(tl_power, tl_prev_power);
            tr_power = slow_accelerate(tr_power, tr_prev_power);
            bl_power = slow_accelerate(bl_power, bl_prev_power);
            br_power = slow_accelerate(br_power, br_prev_power);
        }

        tl.setPower(tl_power);
        tr.setPower(tr_power);
        bl.setPower(bl_power);
        br.setPower(br_power);

        tl_prev_power = tl_power;
        tr_prev_power = tr_power;
        bl_prev_power = bl_power;
        br_prev_power = br_power;

        telemetry.addData("tl motor speed", tl_power);
        telemetry.addData("tr motor speed", tr_power);
        telemetry.addData("bl motor speed", bl_power);
        telemetry.addData("br motor speed", br_power);

        telemetry.addData("angle", angle);

        telemetry.addData("dpad_mode", dpad_mode);
        telemetry.addData("acceleration", accelerate);
    }

    private double slow_accelerate(double power, double prev_power) {
        if (power > prev_power + 0.01 || power < prev_power - 0.01) {
            power += (power - prev_power) / 10;
        }
        return power;
    }

    @Override
    public void stop() {

    }
}
