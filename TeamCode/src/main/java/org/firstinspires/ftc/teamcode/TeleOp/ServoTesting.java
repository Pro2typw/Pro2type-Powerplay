package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants;
import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;

@TeleOp (name = "ServoTesting")

public class
ServoTesting extends LinearOpMode {

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

            if(gamepad2.x) {
                r.intake();
            }

            if(gamepad2.b) {
                r.baseR.setPosition(Constants.rArmIntakePrep);
                r.baseL.setPosition(Constants.lArmIntakePrep);
            }
            if(gamepad2.a) {
                r.baseR.setPosition(Constants.rArmHold);
                r.baseL.setPosition(Constants.lArmHold);
            }
            if(gamepad2.y) {
                r.baseL.setPosition(1);
                r.baseR.setPosition(0);
            }

            adjustingServo(gamepad2.left_stick_y);

        }
    }

    public void adjustingServo(double adjust) {
        double adjustmentL = r.baseL.getPosition() + (adjust * .01);
        double adjustmentR = r.baseR.getPosition() - (adjust * .01);
        r.baseR.setPosition(adjustmentR);
        r.baseL.setPosition(adjustmentL);
    }
}