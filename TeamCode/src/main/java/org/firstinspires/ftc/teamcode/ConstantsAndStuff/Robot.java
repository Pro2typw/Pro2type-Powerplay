package org.firstinspires.ftc.teamcode.ConstantsAndStuff;

import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.LINKAGE_DOWN;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.LINKAGE_HIGH;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.LINKAGE_LOW;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.LINKAGE_MEDIUM;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.lArmIn;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.lArmIntakeLimit;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.lOpen;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.rArmIn;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.rArmIntakePrep;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.rArmintakeLimit;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.rOpen;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBlue;

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
    public double kp = 0.012;
    public double ki = 0.0000401234567;  //0.000030035294;
    public double kd = 0.0008;///9;  //0.000001037255;

    //going down field for pid
    public double kpDown = 0.000004;
    public double kiDown = 0.0000000801234567;
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


    public boolean open = true;

    public boolean firstTime = true;
    public boolean secondTime = false;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        this.fl = hwMap.dcMotor.get("flMec");
        this.fr = hwMap.dcMotor.get("frMec");
        this.br = hwMap.dcMotor.get("brMec");
        this.bl = hwMap.dcMotor.get("blMec");

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

        linkr = hwMap.dcMotor.get("linkager");
        linkl = hwMap.dcMotor.get("linkagel");

        linkl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setDirection(DcMotorSimple.Direction.REVERSE);

        linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        linkr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        telemetry.addData("Intake State", intake);

//        telemetry.addData("Error", error);
//        telemetry.addData("Derivative", derivative);
//        telemetry.addData("Integral Sum", integralSum);

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
            IntakePos = WhereisIntake.HOLD;
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

    public void intakePrep() {
        baseL.setPosition(Constants.lArmIntakePrep);
        baseR.setPosition(Constants.rArmIntakePrep);
    }

    public void adjust(double adjust) {
        double adjustmentL = baseL.getPosition() + (adjust * .01);
        double adjustmentR = baseR.getPosition() - (adjust * .01);

        if(adjustmentL > .73 || adjustmentR < .33){
            adjustmentR = .33;
            adjustmentL = .73;
        }
        if(((adjustmentL > lArmIntakeLimit && adjustmentL < lArmIntakeLimit + .05) || (adjustmentR < rArmintakeLimit && adjustmentR > rArmintakeLimit - .05)) || ((adjustmentL < .56 && adjustmentL > .54) || (adjustmentR > .52 && adjustmentR < .54))) {
            close();
        }
        if(adjustmentL < lArmIn || adjustmentR > rArmIn) {
            adjustmentL = lArmIn;
            adjustmentR = rArmIn;
        }
        baseL.setPosition(adjustmentL);
        baseR.setPosition(adjustmentR);
    }

    public void colorSensorBlue() {
        if(IntakePos == WhereisIntake.CONESTACK) {
            if(state == StateDR4B.START && frontColorSensor.blue() > 900) {
                open = false;
                clawPosition(open);
            }
        }
        else if(state == StateDR4B.START && frontColorSensor.blue() > 250) {
            open = false;
            clawPosition(open);
        }
    }

    public void colorSensorRed() {
        if(state == StateDR4B.START && IntakePos == WhereisIntake.CONESTACK && frontColorSensor.red() > 900) {
            open = false;
            clawPosition(open);
        }
        else if(state == StateDR4B.START && frontColorSensor.red() > 250) {
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
        PREP,
        AUTODOWN
    }

    public StateDR4B state = StateDR4B.START;

    public enum DeployingStateDR4B {
        UP,
        DEPLOY,
        HOLD,
        INTAKE,
        WAIT,
        DOWN
    }

    public DeployingStateDR4B deploying = DeployingStateDR4B.UP;

    public void DR4BState() {

        switch (state) {

            case LOW:

//                hold();
//                if(baseR.getPosition() < .75 && baseL.getPosition() > .32) {
//                    linkageTarget = LINKAGE_LOW;
//                }
//
//                if(getPos(linkl) > -270 && getPos(linkl) < -260 && getPos(linkr) > -270 && getPos(linkr) < 260) {
//                    deploying = DeployingStateDR4B.DEPLOY;
//                }
//
//                if (deploying == DeployingStateDR4B.DEPLOY) {
//                    deploy();
//                    deploying = DeployingStateDR4B.WAIT;
//                }
//
//                if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && clawL.getPosition() >= .85 && clawL.getPosition() <= .87) {
//                    if(deployTimer.milliseconds() > 500) {
//                        open = false;
//                        clawPosition(open);
//                        deploying = DeployingStateDR4B.HOLD;
//                        state = StateDR4B.PREP;
//                    }
//                }
//
//                if(deploying == DeployingStateDR4B.HOLD) {
//                    if(firstTime) {
//                        deployTimer.reset();
//                        firstTime = false;
//                    }
//                    if(deployTimer.milliseconds() > 200) {
//                        hold();
//                        linkageTarget = LINKAGE_LOW;
//                        firstTime = true;
//                    }
//                }

                if(IntakePos == WhereisIntake.HOLD) {
                    linkageTarget = LINKAGE_LOW;
                }


                //getPos(linkl) > -635 && getPos(linkl) < -625 && getPos(linkr) > -635 && getPos(linkr) < -625
                if(getPos(linkl) < -200 && getPos(linkr) < -200 && deploying == DeployingStateDR4B.WAIT) {
                    deploy();
                    deploying = DeployingStateDR4B.DEPLOY;
                }

                if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && clawL.getPosition() >= .85 && clawL.getPosition() <= .87) {
                    if(deployTimer.milliseconds() > 500) {
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
                        firstTime = true;
                        secondTime = true;
                    }
                    if(secondTime) {
                        otherDeployTimer.reset();
                        secondTime = false;
                    }
                    if(otherDeployTimer.milliseconds() > 1000 && (getPos(linkl) > -410 && getPos(linkl) < -400 && getPos(linkr) > -410 && getPos(linkr) < 400)) {
                        linkageTarget = LINKAGE_DOWN;
                        state = StateDR4B.DOWN;

                        deploying = DeployingStateDR4B.DOWN;
                        downTimer.reset();
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

//                hold();//
//                if(baseR.getPosition() < .75 && baseL.getPosition() > .32) {
//                    linkageTarget = LINKAGE_MEDIUM;
//                }
//
//                //getPos(linkl) > -410 && getPos(linkl) < -400 && getPos(linkr) > -410 && getPos(linkr) < 400
//                if(getPos(linkl) < -410 && getPos(linkr) < -410) {
//                    deploying = DeployingStateDR4B.DEPLOY;
//                }
//
//                if (deploying == DeployingStateDR4B.DEPLOY) {
//                    deploy();
//                    deploying = DeployingStateDR4B.WAIT;
//                }
//
//                if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && clawL.getPosition() >= .85 && clawL.getPosition() <= .87) {
//                    if(deployTimer.milliseconds() > 500) {
//                        open = false;
//                        clawPosition(open);
//                        deploying = DeployingStateDR4B.HOLD;
//                        state = StateDR4B.PREP;
//                    }
//                }
//
//                if(deploying == DeployingStateDR4B.HOLD) {
//                    if(firstTime) {
//                        deployTimer.reset();
//                        firstTime = false;
//                    }
//                    if(deployTimer.milliseconds() > 200) {
//                        hold();
//                        linkageTarget = LINKAGE_LOW;
//                        firstTime = true;
//                    }
//                }

                if(IntakePos == WhereisIntake.HOLD) {
                    linkageTarget = LINKAGE_MEDIUM;
                }


                //getPos(linkl) > -635 && getPos(linkl) < -625 && getPos(linkr) > -635 && getPos(linkr) < -625
                if(getPos(linkl) < -350 && getPos(linkr) < -350 && deploying == DeployingStateDR4B.WAIT) {
                    deploy();
                    deploying = DeployingStateDR4B.DEPLOY;
                }

                if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && clawL.getPosition() >= .85 && clawL.getPosition() <= .87) {
                    if(deployTimer.milliseconds() > 500) {
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
                        secondTime = false;
                    }
                    if(otherDeployTimer.milliseconds() > 1000 && (getPos(linkl) > -410 && getPos(linkl) < -400 && getPos(linkr) > -410 && getPos(linkr) < 400)) {
                        linkageTarget = LINKAGE_LOW;

                        deploying = DeployingStateDR4B.DOWN;
                        downTimer.reset();
                    }
                    if(deploying == DeployingStateDR4B.DOWN && downTimer.milliseconds() > 200) {
                        state = StateDR4B.DOWN;
                    }

                }

//                if(deploying == DeployingStateDR4B.HOLD) {
//                    state = StateDR4B.DOWN;
//                }

            break;

            case TOP:

                if(IntakePos == WhereisIntake.HOLD) {
                    linkageTarget = LINKAGE_HIGH;
                }

                //getPos(linkl) > -635 && getPos(linkl) < -625 && getPos(linkr) > -635 && getPos(linkr) < -625
                if(getPos(linkl) < -550 && getPos(linkr) < -550 && deploying == DeployingStateDR4B.WAIT) {
                    deploy();
                    deploying = DeployingStateDR4B.DEPLOY;
                }

                if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && clawL.getPosition() >= .85 && clawL.getPosition() <= .87) {
                    linkageTarget += 20;
                    if(deployTimer.milliseconds() > 500) {
                        open = false;
                        clawPosition(open);
                        deploying = DeployingStateDR4B.HOLD;
                        state = StateDR4B.PREP;
                        firstTime = true;
                        downTimer.reset();
                    }
                }


            break;

            case AUTODOWN:

                if(deploying == DeployingStateDR4B.HOLD) {
                    linkageTarget = LINKAGE_DOWN;

                    deploying = DeployingStateDR4B.DOWN;
//                    downTimer.reset();
//                    }
                    if(deploying == DeployingStateDR4B.DOWN) {
                        downTimer.reset();
                        deploying = DeployingStateDR4B.INTAKE;
                    }

                    if(deploying == DeployingStateDR4B.INTAKE && downTimer.milliseconds() > 1000) {
                        state = StateDR4B.DOWN;
                        firstTime = true;
                        linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linkr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        linkl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        adjustment = 0;
                        linkageTarget = 0;
                        state = Robot.StateDR4B.START;
                        linkl.setPower(0);
                        linkr.setPower(0);
                        intake = Robot.Intake.PREP;
                        intakeTimer.reset();
                    }

                }

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
                    linkr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    linkl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                        state = StateDR4B.AUTODOWN;
                    }
                }
        }
    }


    public enum Intake {
        PREP,
        INTAKE,
        NOTHING
    }

    public Intake intake = Intake.NOTHING;

    public void IntakePos() {
        switch (intake) {
            case PREP:
                intakePrep();
                if(baseR.getPosition() == rArmIntakePrep) {
                    open = true;
                    intake = Intake.INTAKE;
                }
                break;

            case INTAKE:
                intake();
                intake = Intake.NOTHING;
                TeleOpBlue.on = false;
                break;
        }
    }
    public enum WhereisIntake {
        INTAKE,
        HOLD,
        CONESTACK,
        DEPLOY
    }

    public WhereisIntake IntakePos = WhereisIntake.INTAKE;
}
