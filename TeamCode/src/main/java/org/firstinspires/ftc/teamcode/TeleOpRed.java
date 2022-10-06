package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor flMec = hardwareMap.dcMotor.get("flMec");
        DcMotor frMec = hardwareMap.dcMotor.get("frMec");
        DcMotor blMec = hardwareMap.dcMotor.get("blMec");
        DcMotor brMec = hardwareMap.dcMotor.get("brMec");

        frMec.setDirection(DcMotorSimple.Direction.REVERSE);
        brMec.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            flMec.setPower(frontLeftPower);
            blMec.setPower(backLeftPower);
            frMec.setPower(frontRightPower);
            brMec.setPower(backRightPower);
        }
    }
}
