package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "teleopRed")
public class TeleOpRed extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {

        DcMotor fl = hardwareMap.dcMotor.get("flMec");
        DcMotor bl = hardwareMap.dcMotor.get("blMec");
        DcMotor fr = hardwareMap.dcMotor.get("frMec");
        DcMotor br = hardwareMap.dcMotor.get("brMec");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        Servo left = hardwareMap.servo.get("leftClaw");
        Servo right = hardwareMap.servo.get("rightClaw");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setTargetPosition(0);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Declare our motors
        // Make sure your ID's match your configuration


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            int clawNum= 0;
            boolean claw = false;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Non Field Centric
            double motorfl = (y + x + rx) / denominator;
            double motorbl = (y - x + rx) / denominator;
            double motorfr = (y - x - rx) / denominator;
            double motorbr = (y + x - rx) / denominator;


            fl.setPower(motorfl);
            bl.setPower(motorbl);
            fr.setPower(motorfr);
            br.setPower(motorbr);
            armMotor.setPower(-gamepad2.left_stick_y*.7);

            /*if(gamepad2.dpad_down)
            {
                armMotor.setTargetPosition(20);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
            }
            if(gamepad2.dpad_up)
            {
                armMotor.setTargetPosition(180);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.3);
            }
            if(gamepad2.dpad_left)
            {
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(90);
                armMotor.setPower(.7);
            }
            */



            telemetry.addData("you are", "bad");
            // telemetry.addDate("Left Stick Y", y);
            // telemetry.addDate("Right Stick X", rx);
            telemetry.update();


            if(gamepad2.right_bumper)
            {
                left.setPosition(.2);
                right.setPosition(.02);
            }
            if(gamepad2.left_bumper)
            {
                right.setPosition(.08);
                left.setPosition(.14);
            }
        }
    }
}