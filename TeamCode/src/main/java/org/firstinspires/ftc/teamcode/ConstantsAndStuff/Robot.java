package org.firstinspires.ftc.teamcode.ConstantsAndStuff;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Robot {

    public DcMotor fl = null, fr = null, br = null, bl = null, linkr = null, linkl = null;

    public Servo clawR, baseR, baseL, clawL;

    public ColorSensor frontColorSensor = null;

    //public ModernRoboticsI2cColorSensor frontColorSensor = null;

    public HardwareMap hwMap = null;
    public Telemetry telemetry = null;

    private double integralSum = 0;
    private double Ki = .04;
    private double Kd = 0;
    private double Kp = 0.04;
    public double power;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    // intake = the position is down
    public boolean intake;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        this.fl = hwMap.dcMotor.get("flMec");
        this.fr = hwMap.dcMotor.get("frMec");
        this.br = hwMap.dcMotor.get("brMec");
        this.bl = hwMap.dcMotor.get("blMec");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        linkr = hwMap.dcMotor.get("linkager");
        linkl = hwMap.dcMotor.get("linkagel");

//        linkl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        linkr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setDirection(DcMotorSimple.Direction.REVERSE);

        linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linkl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawL = hwMap.servo.get("leftClaw");
        clawR = hwMap.servo.get("rightClaw");

        baseL = hwMap.servo.get("baseL");
        baseR = hwMap.servo.get("baseR");

        frontColorSensor = hwMap.colorSensor.get("Color1");
        frontColorSensor.enableLed(true);

        baseR.setPosition(.98);
        baseL.setPosition(0);

        //public state1 = IntakeHoldStateDR4B.REST;
        //DeployStateDR4B deployState = DeployStateDR4B.REST;

        BNO055IMU imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);


        //webcamInit(hardwareMap);

    }

//    public void webcamInit(HardwareMap ahwMap) {
//        hwMap = ahwMap;
//
//        int cameraMonitorViewId = hwMap.appContext.getResources().
//                getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//
//        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
//        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        //webcam.setPipeline(getClass(SignalDetectionPipeline));
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//    }

//    public void webcamInit(HardwareMap ahwMap) {
//        hwMap = ahwMap;
//
//        int cameraMonitorViewId = hwMap.appContext.getResources().
//                getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//
////        webcamName = hwMap.get(WebcamName.class, "Webcam 1");
////        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
////        webcam.setPipeline(SignalDetectionPipeline);
////        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//
//            @Override
////            public void onOpened() {
////               // webcam.startStreaming(960, 720, OpenCvCameraRotation.UPSIDE_DOWN);
////            }
//
//            @Override
////            public void onError(int errorCode) {
////            }
//
//    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        init(hardwareMap);
        this.telemetry = telemetry;
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

    public void pickUp() {

    }

    public void telemetry() {
        if (telemetry == null) return;
        telemetry.addData("Left Linkage position", linkl.getCurrentPosition());
        telemetry.addData("Left Linkage Power", linkl.getPower());
        telemetry.addData("Right Linkage position", linkr.getCurrentPosition());
        telemetry.addData("Right Linkage Power", linkr.getPower());

        telemetry.addData("Right Claw Pos", clawR.getPosition());
        telemetry.addData("Left Claw Pos", clawL.getPosition());

        telemetry.addData("Left Arm", baseL.getPosition());
        telemetry.addData("RightArm", baseR.getPosition());

        telemetry.addData("red:", frontColorSensor.red());
        telemetry.addData("green:", frontColorSensor.green());
        telemetry.addData("blue:", frontColorSensor.blue());

        telemetry.addData("alpha", frontColorSensor.alpha());

        telemetry.update();
    }

    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.milliseconds();
        double derivative = (error - lastError) / timer.milliseconds();
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double getPos(DcMotor motor) {
        return motor.getCurrentPosition();
    }

    public void rightlinkage(double target, double adjustment) {
        linkr.setPower(PIDController(target + adjustment, getPos(linkr)));
    }

    public void leftlinkage(double target, double adjustment) {
        linkl.setPower(PIDController(target + adjustment, getPos(linkl)));
    }

    public void clawPosition(boolean opened) {
        //
        if (opened) {
            clawR.setPosition(Constants.rOpen);
            clawL.setPosition(Constants.lOpen);
        } else if (!opened) {
            clawR.setPosition(Constants.rClose);
            clawL.setPosition(Constants.lClose);
        }
    }

    public void deploy() {
        baseL.setPosition(Constants.lArmOut);
        baseR.setPosition(Constants.rArmOut);
    }

    public void hold() {
        baseL.setPosition(Constants.lArmHold);
        baseR.setPosition(Constants.rArmHold);
    }

    public void intake() {

        baseL.setPosition(Constants.lArmIn);
        baseR.setPosition(Constants.rArmIn);
    }

    public void adjust(double adjust) {
        double adjustmentL = baseL.getPosition() + (adjust * .005);
        double adjustmentR = baseR.getPosition() - (adjust * .005);
        if(adjustmentL > .7 || adjustmentR < .3){
            adjustmentR = .3;
            adjustmentL = .7;
        }
        if(adjustmentR > .98){
            adjustmentR = .96;
        }

        baseL.setPosition(adjustmentL);
        baseR.setPosition(adjustmentR);
    }

    public void adjustL(double adjust) {
        double justAdjustmentL = baseL.getPosition() + (adjust * .005);
        baseL.setPosition(justAdjustmentL);
    }

    public void adjustR(double adjust) {
        double justAdjustmentR = baseR.getPosition() + (adjust * .005);
        baseR.setPosition(justAdjustmentR);
    }



    public void move() {
        fl.setPower(.1);
        fr.setPower(.1);
        bl.setPower(-.1);
        br.setPower(-.1);
    }

    public void strafeR() {
        fl.setPower(.1);
        fr.setPower(-.1);
        br.setPower(-.1);
        bl.setPower(.1);
    }

    public void strafeL() {
        fl.setPower(-.1);
        fr.setPower(.1);
        br.setPower(.1);
        bl.setPower(-.1);
    }

    public void park() {
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void pos0() {
        baseL.setPosition(0);
        baseR.setPosition(0);
    }

    public void pos1() {
        baseL.setPosition(1);
        baseR.setPosition(1);
    }

    /*public enum IntakeHoldStateDR4B {
        REST

    }

    public enum DeployStateDR4B {
        INTAKE,
        REST,
        GROUND,
        BOTTOM,
        MIDDLE,
        TOP
    }


    public void holdDR4BState() {
        if (intake == false) {
            hold();
        }

        if(state1 == HoldStateDR4B.REST)
        {
            hold();
        }
    }

    public void deployDR4BState() {

        switch (deployState) {

            case DeployStateDR4B.GROUND:

                if(baseL.getPosition() > 0.32  && baseL.getPosition() < 0.34 && baseR.getPosition() < 0.67 && baseR.getPosition() > 0.65)
                {
                    rlinkage(rLinkDown, 0);
                    llinkage(lLinkDown, 0);
                }

                deploy();
                deployState = DeployStateDR4B.REST;

                //how to set rest to hold position here?

            break;

            case DeployStateDR4B.BOTTOM:

                if(baseL.getPosition() > 0.32  && baseL.getPosition() < 0.34 && baseR.getPosition() < 0.67 && baseR.getPosition() > 0.65)
                    {
                        rlinkage(rLinkLow, 0);
                        llinkage(lLinkLow, 0);
                    }
                    deploy();
                    deployState = DeployStateDR4B.REST;

            break;


            case DeployStateDR4B.MIDDLE:

                if(baseL.getPosition() > 0.32  && baseL.getPosition() < 0.34 && baseR.getPosition() < 0.67 && baseR.getPosition() > 0.65)
                    {
                        rlinkage(rLinkMedium, 0);
                        llinkage(lLinkMedium, 0);
                    }
                    deploy();
                    deployState = DeployStateDR4B.REST;

            break;


            case DeployStateDR4B.TOP:

                if(baseL.getPosition() > 0.32  && baseL.getPosition() < 0.34 && baseR.getPosition() < 0.67 && baseR.getPosition() > 0.65)
                    {
                        rlinkage(rLinkHigh, 0);
                        llinkage(lLinkHigh, 0);
                    }
                deploy();
                deployState = DeployStateDR4B.REST;

            break;
        }
    }

     */
}
