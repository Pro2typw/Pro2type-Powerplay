package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot.adjustment;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot.linkageTarget;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;

@TeleOp(name = "DownPIDTuning")
public class DownPIDTuning extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap, telemetry);
        //hardware map = where motors plugged in

        double targetDown = 0;
        int selectedDown = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (selectedDown == 1) robot.kpDown += (gamepad1.right_trigger - gamepad1.left_trigger) * .00001;
            if (selectedDown == 2) robot.kiDown += (gamepad1.right_trigger - gamepad1.left_trigger) * .00000001;
            if (selectedDown == 3) robot.kdDown += (gamepad1.right_trigger - gamepad1.left_trigger) * .0000001;

            if (gamepad1.a) selectedDown = 1;
            if (gamepad1.b) selectedDown = 2;
            if (gamepad1.x) selectedDown = 3;

            if (gamepad1.right_bumper) robot.integralSumDown = 0;

            robot.adjust(gamepad2.right_stick_x);

            adjustment += gamepad1.right_stick_y * 3;
//            robot.linkagePower(target, 0);

            telemetry.addData("currently adjusting", selectedDown);
            telemetry.addData("target", targetDown);
            telemetry.addData("left position", robot.getPos(robot.linkl));
            telemetry.addData("right position", robot.getPos(robot.linkr));
            telemetry.addData("kP", robot.kpDown);
            telemetry.addData("kI", robot.kiDown);
            telemetry.addData("kD", robot.kdDown);
            telemetry.addData("P error", robot.errorDown);
            telemetry.addData("I error", robot.integralSumDown);
            telemetry.addData("D error", robot.derivativeDown);
            telemetry.update();
            sleep(10);

            if (gamepad2.dpad_down) {
                adjustment = 0;
                robot.state = Robot.StateDR4B.DOWN;
                robot.DR4BState();
            }
            if (gamepad2.dpad_up) {
                adjustment = 0;
                robot.state = Robot.StateDR4B.TOP;
                robot.DR4BState();
            }
            if (gamepad2.dpad_left) {
                adjustment = 0;
                robot.state = Robot.StateDR4B.BOTTOM;
                robot.DR4BState();
            }
            if(gamepad2.dpad_right) {
                adjustment = 0;
                robot.state = Robot.StateDR4B.MIDDLE;
                robot.DR4BState();
            }
            robot.linkagePower(linkageTarget, adjustment);
        }
    }
}
