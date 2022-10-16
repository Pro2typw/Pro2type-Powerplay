package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum TeleOp Field-Centric")  // TODO: Add preselectTeleOp accordingly with driver preference ... https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/eventloop/opmode/Autonomous.html
public class MecanumTeleOpFieldCentric extends LinearOpMode {
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

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();
        if(isStopRequested()) return;

        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double lfPower = ((rotY + rotX + rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;
            double lbPower = ((rotY - rotX + rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;
            double rfPower = ((rotY - rotX - rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;
            double rbPower = ((rotY + rotX - rx) / denominator) * Constants.DriveTrainConstants.MAX_SPEED;

            lf.setPower(lfPower);
            lb.setPower(lbPower);
            rf.setPower(rfPower);
            rb.setPower(rbPower);

        }
    }
}
