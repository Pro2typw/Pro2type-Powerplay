package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "DR4B testing")


public class DR4BTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor linkr = hardwareMap.dcMotor.get("linkager");
        DcMotor linkl = hardwareMap.dcMotor.get("linkagel");

        linkr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linkl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linkl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkr.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("left encoder", linkl.getCurrentPosition());
            telemetry.addData("right encoder", linkl.getCurrentPosition());
            telemetry.update();
            linkr.setPower(gamepad2.left_stick_y);
            linkl.setPower(gamepad2.left_stick_y);
        }

    }

}
