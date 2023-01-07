package org.firstinspires.ftc.teamcode.ConstantsAndStuff;

import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.LINKAGE_DOWN;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.LINKAGE_HIGH;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.LINKAGE_LOW;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.LINKAGE_MEDIUM;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.lArmIn;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.lOpen;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.rArmIn;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.rOpen;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot{

    public DcMotor fl = null, fr = null, br = null, bl = null, linkr = null, linkl = null;

    public Servo clawR, baseR, baseL, clawL;

    public ColorSensor frontColorSensor = null;

    //public ModernRoboticsI2cColorSensor frontColorSensor = null;

    public HardwareMap hwMap = null;
    public Telemetry telemetry = null;


    public static int adjustment = 0;
    public static int linkageTarget = 0;

    //going up field for pid
    public double kp = 0.016;
    public double ki = 0.000038835294;
    public double kd = 0.0100437255;

    //going down field for pid
    public double kpDown = 0.011;
    public double kiDown = 0.00003035294;
    public double kdDown = 0.00010437255;

    //going up field for pid calculations variables
    public double error = 0;
    public double derivative = 0;
    public double integralSum = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public ElapsedTime deployTimer = new ElapsedTime();
    public ElapsedTime otherDeployTimer = new ElapsedTime();

    //going down field for pid calculations variables
    public double errorDown = 0;
    public double derivativeDown = 0;
    public double integralSumDown = 0.0;
    private ElapsedTime timerDown = new ElapsedTime();
    private double lastErrorDown = 0;


    public boolean open = true;

    public boolean firstTime = true;
    public boolean secondTime = false;

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

        linkl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        baseR.setPosition(rArmIn);
        baseL.setPosition(lArmIn);

        clawR.setPosition(rOpen);
        clawL.setPosition(lOpen);



        BNO055IMU imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        adjustment = 0;
        linkageTarget = 0;

        state = StateDR4B.START;


        //webcamInit(hardwareMap);

    }

    public void initDR4B(HardwareMap ahwMap, Telemetry telemetry) {
        linkr = hwMap.dcMotor.get("linkager");
        linkl = hwMap.dcMotor.get("linkagel");

        linkl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setDirection(DcMotorSimple.Direction.REVERSE);

        linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linkl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;

    }

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

        telemetry.addData("Linkage target", linkageTarget);
        telemetry.addData("Linkage adjustment", adjustment);
        telemetry.addData("PID output", PIDController(linkageTarget + adjustment, linkl.getCurrentPosition()));

        telemetry.addData("Right Claw Pos", clawR.getPosition());
        telemetry.addData("Left Claw Pos", clawL.getPosition());

        telemetry.addData("Left Arm", baseL.getPosition());
        telemetry.addData("RightArm", baseR.getPosition());

        telemetry.addData("State", state);

        telemetry.addData("Error", error);
        telemetry.addData("Derivative", derivative);
        telemetry.addData("Integral Sum", integralSum);

        telemetry.addData("blue", frontColorSensor.blue());
        telemetry.addData("red", frontColorSensor.red());

        telemetry.update();
    }

    //reference: where you want to go
    //state: where it currently is
    public double PIDController(double reference, double state) {
        error = reference - state;
        integralSum += error * timer.milliseconds();
        derivative = (error - lastError) / timer.milliseconds();
        lastError = error;
        timer.reset();
        return (error * kp) + (integralSum * ki) + (derivative * kd);
    }

    public double PIDControllerDown(double reference, double state) {
        errorDown = reference - state;
        integralSumDown += errorDown * timerDown.milliseconds();
        derivativeDown = (errorDown - lastErrorDown) / timerDown.milliseconds();
        lastErrorDown = errorDown;
        timerDown.reset();
        return (errorDown * kpDown) - (derivativeDown * kdDown) + (integralSumDown * kiDown);
    }

    public double getPos(DcMotor motor) {
        return motor.getCurrentPosition();
    }

    public void linkagePower(double target, double adjustment) {
        linkr.setPower(PIDController(target + adjustment, getPos(linkr)));
        linkl.setPower(PIDController(target + adjustment, getPos(linkl)));
    }

    public void linkagePowerDown(double target, double adjustment) {
        linkr.setPower(PIDControllerDown(target + adjustment, getPos(linkr)));
        linkl.setPower(PIDControllerDown(target + adjustment, getPos(linkl)));
    }

    public void clawPosition(boolean opened) {
        //
        if (opened) {
            clawR.setPosition(Constants.rOpen);
            clawL.setPosition(lOpen);
        } else if (!opened) {
            clawR.setPosition(Constants.rClose);
            clawL.setPosition(Constants.lClose);
            hold();
        }
    }

    public void close() {
        clawR.setPosition(Constants.rClose);
        clawL.setPosition(Constants.lClose);
        open = false;
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
        double adjustmentL = baseL.getPosition() + (adjust * .01);
        double adjustmentR = baseR.getPosition() - (adjust * .01);
        if(adjustmentL < .078) {
            adjustmentL = .078;
        }
        if(adjustmentL > .73 || adjustmentR < .33){
            adjustmentR = .33;
            adjustmentL = .73;
        }
        if(((adjustmentL > .19 && adjustmentL < .21) || (adjustmentR < .87 && adjustmentR > .85)) || ((adjustmentL < .56 && adjustmentL > .54) || (adjustmentR > .52 && adjustmentR < .54))) {
            open = false;
            clawPosition(open);
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

    public void pos0() {
        baseL.setPosition(0);
        baseR.setPosition(0);
    }

    public void pos1() {
        baseL.setPosition(1);
        baseR.setPosition(1);
    }

    public void colorSensor() {
        if(state == StateDR4B.START && baseL.getPosition() > 0.097 && baseL.getPosition() < 0.099 && baseR.getPosition() > 0.97 && baseR.getPosition() < 0.99 && frontColorSensor.blue() > 600) {
            open = false;
            clawPosition(open);
        }
    }

    public enum StateDR4B {
        START,
        LOW,
        MIDDLE,
        TOP,
        //go to hold and then down to intake pos
        DOWN,
        ADJUSTMENT,
        PREP
    }

    public StateDR4B state = StateDR4B.START;

    public enum DeployingStateDR4B {
        UP,
        DEPLOY,
        HOLD,
        INTAKE,
        WAIT
    }

    public DeployingStateDR4B deploying = DeployingStateDR4B.UP;

    public void DR4BState() {

        switch (state) {

            case LOW:

                hold();
                linkageTarget = LINKAGE_LOW;

                if(getPos(linkl) > -270 && getPos(linkl) < -260 && getPos(linkr) > -270 && getPos(linkr) < 260) {
                    deploying = DeployingStateDR4B.DEPLOY;
                }

                if (deploying == DeployingStateDR4B.DEPLOY) {
                    deploy();
                    deploying = DeployingStateDR4B.WAIT;
                }

                if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && clawL.getPosition() >= .85 && clawL.getPosition() <= .87) {
                    if(deployTimer.milliseconds() > 1000) {
                        open = false;
                        clawPosition(open);
                        deploying = DeployingStateDR4B.HOLD;
                        state = StateDR4B.PREP;
                    }
                }

                if(deploying == DeployingStateDR4B.HOLD) {
                    if(firstTime) {
                        deployTimer.reset();
                        firstTime = false;
                    }
                    if(deployTimer.milliseconds() > 200) {
                        hold();
                        linkageTarget = LINKAGE_LOW;
                        firstTime = true;
                    }
                }

//                if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && clawL.getPosition() >= .85 && clawL.getPosition() <= .87) {
//                    deploying = DeployingStateDR4B.HOLD;
//                    state = StateDR4B.PREP;
//                    adjust(-10);
//                    linkageTarget = LINKAGE_LOW;
//                }

            break;

            case MIDDLE:

                hold();
                linkageTarget = LINKAGE_MEDIUM;

                //getPos(linkl) > -410 && getPos(linkl) < -400 && getPos(linkr) > -410 && getPos(linkr) < 400
                if(getPos(linkl) < -410 && getPos(linkr) < -410) {
                    deploying = DeployingStateDR4B.DEPLOY;
                }

                if (deploying == DeployingStateDR4B.DEPLOY) {
                    deploy();
                    deploying = DeployingStateDR4B.WAIT;
                }

                if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && clawL.getPosition() >= .85 && clawL.getPosition() <= .87) {
                    if(deployTimer.milliseconds() > 1000) {
                        open = false;
                        clawPosition(open);
                        deploying = DeployingStateDR4B.HOLD;
                        state = StateDR4B.PREP;
                    }
                }

                if(deploying == DeployingStateDR4B.HOLD) {
                    if(firstTime) {
                        deployTimer.reset();
                        firstTime = false;
                    }
                    if(deployTimer.milliseconds() > 200) {
                        hold();
                        linkageTarget = LINKAGE_LOW;
                        firstTime = true;
                    }
                }

//                if(deploying == DeployingStateDR4B.HOLD) {
//                    state = StateDR4B.DOWN;
//                }

            break;

            case TOP:

                hold();
                linkageTarget = LINKAGE_HIGH;

                //getPos(linkl) > -635 && getPos(linkl) < -625 && getPos(linkr) > -635 && getPos(linkr) < -625
                if(getPos(linkl) < -585 && getPos(linkr) < -585) {
                    deploying = DeployingStateDR4B.DEPLOY;
                }

                if (deploying == DeployingStateDR4B.DEPLOY) {
                    deploy();
                    deploying = DeployingStateDR4B.WAIT;
                }

                if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && clawL.getPosition() >= .85 && clawL.getPosition() <= .87) {
                    if(deployTimer.milliseconds() > 1000) {
                        open = false;
                        clawPosition(open);
                        deploying = DeployingStateDR4B.HOLD;
                        state = StateDR4B.PREP;
                    }
                }

                if(deploying == DeployingStateDR4B.HOLD) {
                    if(firstTime) {
                        deployTimer.reset();
                        firstTime = false;
                    }
                    if(deployTimer.milliseconds() > 200) {
                        hold();
                        linkageTarget = LINKAGE_MEDIUM;
                        firstTime = true;
                        secondTime = true;
                    }
                    if(secondTime) {
                        otherDeployTimer.reset();
                    }
                    if(otherDeployTimer.milliseconds() > 1000 && (getPos(linkl) > -410 && getPos(linkl) < -400 && getPos(linkr) > -410 && getPos(linkr) < 400)) {
                        linkageTarget = LINKAGE_LOW;
                    }
                }

//                if(deploying == DeployingStateDR4B.HOLD) {
//                    state = StateDR4B.DOWN;
//                }

            break;

            case DOWN:

                open = false;
                clawPosition(open);
                hold();
                linkageTarget = LINKAGE_DOWN;
                deployTimer.reset();
                deployTimer.startTime();

                if (deployTimer.milliseconds() > 750) {
                    state = StateDR4B.START;
                    linkageTarget = 0;
                    linkr.setTargetPosition(0);
                    linkr.setPower(.06);
                    linkl.setTargetPosition(0);
                    linkl.setPower(.06);
                    state = StateDR4B.DOWN;

                }

                if (getPos(linkr) >= 0 && getPos(linkl) >= 0) {
                    linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    linkr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linkl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linkageTarget = 0;
                    adjustment = 0;
                    linkr.setPower(0);
                    linkl.setPower(0);
                    state = StateDR4B.START;
                    firstTime = false;
                }

                break;

            case PREP:

                if(deploying == DeployingStateDR4B.HOLD) {
                    if(firstTime) {
                        deployTimer.reset();
                        firstTime = false;
                    }
                    if(deployTimer.milliseconds() > 200) {
                        hold();
                        linkageTarget = LINKAGE_LOW;
                        firstTime = true;
                    }
                }
        }
    }
}
