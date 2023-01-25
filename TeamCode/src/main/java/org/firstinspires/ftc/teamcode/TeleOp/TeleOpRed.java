package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.lArmIntakePrep;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants.rArmIntakePrep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;

@TeleOp (name = "TeleOpBlue")

public class TeleOpBlue extends LinearOpMode {

    Robot r = new Robot();

    boolean deployed = false;

    public static boolean on = false;


    @Override
    public void runOpMode() throws InterruptedException {

        r.init(hardwareMap, telemetry);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // telemetry
            r.telemetry();

            if(r.state == Robot.StateDR4B.AUTODOWN) {
                r.linkagePowerDown(r.linkageTarget, r.adjustment);
            }
            else if(r.state == Robot.StateDR4B.DOWN) {
                r.linkagePowerDown(r.linkageTarget, r.adjustment);
            }
            else if(!(r.state == Robot.StateDR4B.START)) {
                r.linkagePower(r.linkageTarget, r.adjustment);
            }

            //drive
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (r.state == Robot.StateDR4B.TOP || r.state == Robot.StateDR4B.MIDDLE || r.state == Robot.StateDR4B.LOW)
                r.mecanumDrive(y, x, rx, 0.3);
            else
                r.mecanumDrive(y, x, rx, 1.0);

            //The controls for the claw
            if(gamepad2.right_bumper){
                r.open = false;
            }
            else if(gamepad2.left_bumper){
                r.open = true;
                r.deployTimer.reset();
            }
            if(gamepad2.right_bumper || gamepad2.left_bumper) {
                r.clawPosition(r.open);
            }
            r.colorSensorRed();

            if(r.state == Robot.StateDR4B.AUTODOWN) {
                r.linkagePowerDown(r.linkageTarget, r.adjustment);
            }
            else if(r.state == Robot.StateDR4B.DOWN) {
                r.linkagePowerDown(r.linkageTarget, r.adjustment);
            }
            else if(!(r.state == Robot.StateDR4B.START)) {
                r.linkagePower(r.linkageTarget, r.adjustment);
            }


            //The controls for the arm
            if(gamepad2.b){
                r.open = false;
                r.clawPosition(r.open);
                r.deploy();
            }

            else if(gamepad2.x && r.baseL.getPosition() < .4){
                r.intake = Robot.Intake.PREP;
                r.intakeTimer.reset();
            }
            else if(gamepad2.y){
                r.open = false;
                r.clawPosition(r.open);
                r.hold();
            }
            else if(gamepad2.a) {
                r.baseR.setPosition(rArmIntakePrep);
                r.baseL.setPosition(lArmIntakePrep);
            }

            r.adjust(gamepad2.right_stick_x);


            if(r.intake == Robot.Intake.PREP){
                r.intakePrep();
                if(r.intakeTimer.milliseconds() > 600) {
                    if(r.baseR.getPosition() == rArmIntakePrep) {
                        r.open = true;
                        r.clawPosition(r.open);
                        r.intake = Robot.Intake.INTAKE;
                    }
                }
            }
            else if(r.intake == Robot.Intake.INTAKE) {
                r.intake();
                r.intake = Robot.Intake.NOTHING;
                TeleOpBlue.on = false;
            }



            if(r.state == Robot.StateDR4B.AUTODOWN) {
                r.linkagePowerDown(r.linkageTarget, r.adjustment);
            }
            else if(r.state == Robot.StateDR4B.DOWN) {
                r.linkagePowerDown(r.linkageTarget, r.adjustment);
            }
            else if(!(r.state == Robot.StateDR4B.START)) {
                r.linkagePower(r.linkageTarget, r.adjustment);
            }

            //driver controls for DR4B state

            if (gamepad2.dpad_down) {
                r.adjustment = 0;
                r.state = Robot.StateDR4B.DOWN;
            }
            if (gamepad2.dpad_up) {
                r.open = false;
                r.clawPosition(r.open);
                r.hold();
                r.adjustment = 0;
                r.state = Robot.StateDR4B.TOP;
                r.deploying = Robot.DeployingStateDR4B.WAIT;
            }
            if (gamepad2.dpad_left) {
                r.open = false;
                r.clawPosition(r.open);
                r.hold();
                r.adjustment = 0;
                r.state = Robot.StateDR4B.LOW;
                r.deploying = Robot.DeployingStateDR4B.WAIT;
            }
            if(gamepad2.dpad_right) {
                r.open = false;
                r.clawPosition(r.open);
                r.hold();
                r.adjustment = 0;
                r.state = Robot.StateDR4B.MIDDLE;
                r.deploying = Robot.DeployingStateDR4B.WAIT;
            }

            if(!(r.state == Robot.StateDR4B.LOW || r.state == Robot.StateDR4B.MIDDLE || r.state == Robot.StateDR4B.TOP) && Math.abs(gamepad2.left_stick_y) > .1) {
                if(r.state == Robot.StateDR4B.DOWN) {
                    r.adjustment = (int)((r.getPos(r.linkl) + r.getPos(r.linkr)) / 2);
                }
                r.state = Robot.StateDR4B.ADJUSTMENT;
                r.adjustment += (int) (gamepad2.left_stick_y * 5);
            }
            else {
                r.adjustment += (int) (gamepad2.left_stick_y * 5);
            }

            if(r.state == Robot.StateDR4B.AUTODOWN) {
                r.linkagePowerDown(r.linkageTarget, r.adjustment);
            }
            else if(r.state == Robot.StateDR4B.DOWN) {
                r.linkagePowerDown(r.linkageTarget, r.adjustment);
            }
            else if(!(r.state == Robot.StateDR4B.START)) {
                r.linkagePower(r.linkageTarget, r.adjustment);
            }

            r.DR4BState();

            if(gamepad2.left_stick_button) {
                r.linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                r.linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                r.linkr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                r.linkl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                r.firstTime = true;
                r.adjustment = 0;
                r.linkageTarget = 0;
                r.state = Robot.StateDR4B.START;
                r.linkl.setPower(0);
                r.linkr.setPower(0);
                r.open = true;

            }

//            r.linkl.setPower(gamepad2.left_stick_y * .175);
//            r.linkr.setPower(gamepad2.left_stick_y * .175);

            if(r.state == Robot.StateDR4B.START) {
                r.linkl.setPower(gamepad2.right_trigger * .15);
                r.linkr.setPower(gamepad2.right_trigger * .15);
            }
        }
    }
}