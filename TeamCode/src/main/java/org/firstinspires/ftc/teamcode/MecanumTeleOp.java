package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumTeleOp")
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lf = hardwareMap.dcMotor.get(Constants.DriveTrainConstants.LF_MAP_NAME);
        DcMotor lb = hardwareMap.dcMotor.get(Constants.DriveTrainConstants.LB_MAP_NAME);
        DcMotor rf = hardwareMap.dcMotor.get(Constants.DriveTrainConstants.RF_MAP_NAME);
        DcMotor rb = hardwareMap.dcMotor.get(Constants.DriveTrainConstants.RB_MAP_NAME);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: Add Additional Reverse Direction if Problems

        lf.setZeroPowerBehavior(Constants.DriveTrainConstants.ZERO_POWER_BEHAVIOR);
        lb.setZeroPowerBehavior(Constants.DriveTrainConstants.ZERO_POWER_BEHAVIOR);
        rf.setZeroPowerBehavior(Constants.DriveTrainConstants.ZERO_POWER_BEHAVIOR);
        rb.setZeroPowerBehavior(Constants.DriveTrainConstants.ZERO_POWER_BEHAVIOR);

        waitForStart();
        if(isStopRequested()) return;

        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double lfPower = ((y + x + rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;
            double lbPower = ((y - x + rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;
            double rfPower = ((y - x - rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;
            double rbPower = ((y + x - rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;

            lf.setPower(lfPower);
            lb.setPower(lbPower);
            rf.setPower(rfPower);
            rb.setPower(rbPower);
        }
    }
}
