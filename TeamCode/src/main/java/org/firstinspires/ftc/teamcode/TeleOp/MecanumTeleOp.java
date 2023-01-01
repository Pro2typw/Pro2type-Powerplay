package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot.adjustment;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot.linkageTarget;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;

@TeleOp (name = "MecanumTeleOp")

public class MecanumTeleOp extends LinearOpMode {

    Robot r = new Robot();

    boolean deployed = false;


    @Override
    public void runOpMode() throws InterruptedException {

        r.init(hardwareMap, telemetry);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // telemetry
            r.telemetry();

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
            }
            if(gamepad2.right_bumper || gamepad2.left_bumper) {
                r.clawPosition(r.open);
            }


            //The controls for the arm
            if(gamepad2.b){
                r.open = false;
                r.clawPosition(r.open);
                r.deploy();
            }
            else if(gamepad2.x){
                r.open = true;
                r.clawPosition(r.open);
                r.intake();
            }
            else if(gamepad2.y){
                r.open = false;
                r.clawPosition(r.open);
                r.hold();
            }
            r.adjust(gamepad2.right_stick_x);

            //driver controls for DR4B state

            if (gamepad2.dpad_down) {
                adjustment = 0;
                r.state = Robot.StateDR4B.DOWN;
            }
            if (gamepad2.dpad_up) {
                adjustment = 0;
                r.state = Robot.StateDR4B.TOP;
            }
            if (gamepad2.dpad_left) {
                r.open = false;
                r.clawPosition(r.open);
                r.hold();
                adjustment = 0;
                r.state = Robot.StateDR4B.LOW;
            }
            if(gamepad2.dpad_right) {
                adjustment = 0;
                r.state = Robot.StateDR4B.MIDDLE;
            }

            if(Math.abs(gamepad2.left_stick_y) > .1) {
                r.state = Robot.StateDR4B.ADJUSTMENT;
                adjustment += (int) (gamepad2.left_stick_y * 2);
            }

            if(!(r.state == Robot.StateDR4B.DOWN || r.state == Robot.StateDR4B.START)) {
                r.linkagePower(linkageTarget, adjustment);
            }
            r.DR4BState();
//            r.linkl.setPower(gamepad2.left_stick_y * .5);
//            r.linkr.setPower(gamepad2.left_stick_y * .5);

        }
    }

}