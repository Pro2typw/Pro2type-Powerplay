package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MecanumTeleOp") // TODO: Add preselectTeleOp accordingly with driver preference ... https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/eventloop/opmode/Autonomous.html
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lf = hardwareMap.dcMotor.get(Constants.DriveTrainConstants.LF_MAP_NAME);
        DcMotor lb = hardwareMap.dcMotor.get(Constants.DriveTrainConstants.LB_MAP_NAME);
        DcMotor rf = hardwareMap.dcMotor.get(Constants.DriveTrainConstants.RF_MAP_NAME);
        DcMotor rb = hardwareMap.dcMotor.get(Constants.DriveTrainConstants.RB_MAP_NAME);
        //DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");


        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: Add Additional Reverse Direction if Problems

        lf.setZeroPowerBehavior(Constants.DriveTrainConstants.ZERO_POWER_BEHAVIOR);
        lb.setZeroPowerBehavior(Constants.DriveTrainConstants.ZERO_POWER_BEHAVIOR);
        rf.setZeroPowerBehavior(Constants.DriveTrainConstants.ZERO_POWER_BEHAVIOR);
        rb.setZeroPowerBehavior(Constants.DriveTrainConstants.ZERO_POWER_BEHAVIOR);
        //armMotor.setZeroPowerBehavior(Constants.DriveTrainConstants.ZERO_POWER_BEHAVIOR);
        Servo left = hardwareMap.servo.get("leftClaw");
        Servo right = hardwareMap.servo.get("rightClaw");

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if(isStopRequested()) return;

        while(opModeIsActive()) {

            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x; //great code dhanush !

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double lfPower = ((y + x + rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;
            double lbPower = ((y - x + rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;
            double rfPower = ((y - x - rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;
            double rbPower = ((y + x - rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;

            lf.setPower(lfPower);
            lb.setPower(lbPower);
            rf.setPower(rfPower);
            rb.setPower(rbPower);
            //armMotor.setPower(gamepad2.left_stick_y);
            /*
            if(gamepad1.dpad_down)
            {
                armMotor.setTargetPosition(0);
            }
            if(gamepad1.dpad_up)
            {
                armMotor.setTargetPosition(90);
            }
            if(gamepad1.dpad_left)
            {
                armMotor.setTargetPosition(45);
            }
            */

            if(gamepad1.a)
            {
                if(claw == true)
                {
                    claw = false;
                }
                else if(claw == false)
                {
                    claw = true;
                }
            }

            while(claw == false)
            {
                left.setPosition(.1);
                right.setPosition(.1);
            }
            while(claw == true)
            {
                left.setPosition(.2);
                right.setPosition(0);
            }

        }
    }
}
