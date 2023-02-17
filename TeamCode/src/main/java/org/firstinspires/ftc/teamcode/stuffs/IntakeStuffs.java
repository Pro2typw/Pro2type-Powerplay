package org.firstinspires.ftc.teamcode.stuffs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeStuffs {

    private RobotStates robotState;

    public DcMotor fl, fr, br, bl, linkr, linkl;
    public Servo clawR, baseR, baseL, clawL;
    public ColorSensor frontColorSensor;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    //going up field for pid
    public double kp = 0.012;
    public double ki = 0.0000401234567;  //0.000030035294;
    public double kd = 0.0008;///9;  //0.000001037255;

    //going down field for pid
    public double kpDown = 0.0000002;
    public double kiDown = 0.000000000801234567;
    public double kdDown = 0.000008;

    //going up field for pid calculations variables
    public double error = 0;
    public double derivative = 0;
    public double integralSum = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public ElapsedTime deployTimer = new ElapsedTime();
    public ElapsedTime otherDeployTimer = new ElapsedTime();
    public ElapsedTime downTimer = new ElapsedTime();
    public ElapsedTime intakeTimer = new ElapsedTime();

    //going down field for pid calculations variables
    public double errorDown = 0;
    public double derivativeDown = 0;
    public double integralSumDown = 0.0;
    private ElapsedTime timerDown = new ElapsedTime();
    private double lastErrorDown = 0;

    public boolean firstTime = true;
    public boolean secondTime = false;

    BNO055IMU imu;
    BNO055IMU.Parameters parameters;


    public IntakeStuffs(HardwareMap hardwareMap, Telemetry telemetry) {
        fl = hardwareMap.dcMotor.get("flMec");
        fr = hardwareMap.dcMotor.get("frMec");
        br = hardwareMap.dcMotor.get("brMec");
        bl = hardwareMap.dcMotor.get("blMec");

        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        linkr = hardwareMap.dcMotor.get("linkager");
        linkl = hardwareMap.dcMotor.get("linkagel");

        linkl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setDirection(DcMotorSimple.Direction.REVERSE);

        linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawL = hardwareMap.servo.get("leftClaw");
        clawR = hardwareMap.servo.get("rightClaw");

        baseL = hardwareMap.servo.get("baseL");
        baseR = hardwareMap.servo.get("baseR");

        frontColorSensor = hardwareMap.colorSensor.get("Color1");
        frontColorSensor.enableLed(true);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
    }

    public void setRobotState() {
        robotState = runState();
    }
    private RobotStates runState() {
        RobotStates returnState = robotState;
        switch (robotState) {
            case INIT:
                // CLOSE CLAW
                // GO TO HOLD POSITION
                // GO TO DR4B LOW
                // GO TO

        }
        return RobotStates.STACKED_INTAKE;
    }

}
