package org.firstinspires.ftc.teamcode.ConstantsAndStuff;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {

    public DcMotor fl = null, fr = null, br = null, bl = null, linkr = null, linkl = null;

    public Servo clawR, baseR, baseL, clawL;

    public HardwareMap hwMap = null;

    public double integralSum = 0;
    public double Ki = 0;
    public double Kd = 0;
    public double Kp = 0.04;
    public double power;
    public int target = 0;
    public int adjustment = 0;
    public ElapsedTime timer = new ElapsedTime();
    public private double lastError = 0;


    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        this.fl = hwMap.dcMotor.get("flMec");
        this.fr = hwMap.dcMotor.get("frMec");
        this.br = hwMap.dcMotor.get("brMec");
        this.bl = hwMap.dcMotor.get("blMec");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        linkr = hwMap.dcMotor.get("linkager");
        linkl = hwMap.dcMotor.get("linkagel");

        linkl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setDirection(DcMotorSimple.Direction.REVERSE);

        clawL = hwMap.servo.get("leftClaw");
        clawR = hwMap.servo.get("rightClaw");

        baseL = hwMap.servo.get("baseL");
        baseR = hwMap.servo.get("baseR");

        BNO055IMU imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    public void mecanumDrive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        fl.setPower((y + x + rx) / denominator);
        bl.setPower((y - x + rx) / denominator);
        fr.setPower((y - x - rx) / denominator);
        br.setPower((y + x - rx) / denominator);
    }

    public void mecanumDrive(double y, double x, double rx, double multiplier) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        fl.setPower(((y + x + rx) / denominator) * multiplier);
        bl.setPower(((y - x + rx) / denominator) * multiplier);
        fr.setPower(((y - x - rx) / denominator) * multiplier);
        br.setPower(((y + x - rx) / denominator) * multiplier);
    }

    public void pickUp(){

    }

    public void telemetry(){
        telemetry.addData("you are", "bad");
        telemetry.addData("Left Linkage position", linkl.getCurrentPosition());
        telemetry.addData("Left Linkage", linkl.getPower());
        telemetry.addData("Right Linkage position", linkr.getCurrentPosition());
        telemetry.addData("Right Linkage power", linkr.getPower());
        telemetry.addData("Right Claw Pos", clawR.getPosition());
        telemetry.addData("Left Claw Pos", clawL.getPosition());
        telemetry.addData("Left Arm", baseL.getPosition());
        telemetry.addData("RightArm", baseR.getPosition());

        telemetry.update();
    }

    public double PIDController(double reference, double state)
    {
        double error = reference - state;
        integralSum += error * timer.milliseconds();
        double derivative = (error - lastError) / timer.milliseconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double getPos(DcMotor motor){
        return motor.getCurrentPosition();
    }

}
