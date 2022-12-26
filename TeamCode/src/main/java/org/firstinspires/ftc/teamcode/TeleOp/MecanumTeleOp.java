package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants;
import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;

@TeleOp (name = "MecanumTeleOp")

public class MecanumTeleOp extends LinearOpMode {

    Robot r = new Robot();


    boolean deployed = false;
    boolean open = false;

    public int rtarget = 0;
    public int ltarget = 0;
    public int adjustment = 0;

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

            if (gamepad1.right_trigger > .3)
                r.mecanumDrive(y, x, rx, 0.5);
            else
                r.mecanumDrive(y, x, rx, 1.0);



            if (gamepad2.dpad_down) {
                rtarget = Constants.rLinkDown;
                ltarget = Constants.lLinkDown;
                adjustment = 0;
            }
            if (gamepad2.dpad_up) {
                rtarget = Constants.rLinkHigh;
                ltarget = Constants.lLinkHigh;
                adjustment = 0;
            }
            if (gamepad1.dpad_left) {
                rtarget = Constants.rLinkMedium;
                ltarget = Constants.lLinkMedium;
                adjustment = 0;
            }
            if(gamepad2.dpad_right) {
                rtarget = Constants.rLinkLow;
                ltarget = Constants.lLinkLow;
                adjustment = 0;
            }

            adjustment += (int) (gamepad2.left_stick_y * 4);

            r.rlinkage(rtarget, adjustment);
            r.llinkage(ltarget, adjustment);


            //The controls for the claw
            if(gamepad2.right_bumper){
                open = false;
            }
            else if(gamepad2.left_bumper){
                open = true;
            }
            r.clawPosition(open);

            //The controls for the arm
            if(gamepad2.b){
                r.deploy();
            }
            else if(gamepad2.x){
                r.intake();
            }
            else if(gamepad2.y){
                r.hold();
            }
            r.adjust(gamepad2.right_stick_x);
            r.adjustL(gamepad2.left_trigger);
            r.adjustR(gamepad2.right_trigger);
        }
    }
}