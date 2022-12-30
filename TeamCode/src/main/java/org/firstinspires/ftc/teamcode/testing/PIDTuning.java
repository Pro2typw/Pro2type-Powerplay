package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot.adjustment;
import static org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot.linkageTarget;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;

@TeleOp(name = "PIDTuning")
public class PIDTuning extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap, telemetry);

        double target = 0;
        int selected = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (selected == 1) robot.kp += (gamepad1.right_trigger - gamepad1.left_trigger) * .00001;
            if (selected == 2) robot.ki += (gamepad1.right_trigger - gamepad1.left_trigger) * .00000001;
            if (selected == 3) robot.kd += (gamepad1.right_trigger - gamepad1.left_trigger) * .0000001;

            if (gamepad1.a) selected = 1;
            if (gamepad1.b) selected = 2;
            if (gamepad1.x) selected = 3;

            if (gamepad1.right_bumper) robot.integralSum = 0;

            robot.adjust(gamepad2.right_stick_x);

            adjustment += gamepad1.right_stick_y * 3;
//            robot.linkagePower(target, 0);

            telemetry.addData("currently adjusting", selected);
            telemetry.addData("target", target);
            telemetry.addData("left position", robot.getPos(robot.linkl));
            telemetry.addData("right position", robot.getPos(robot.linkr));
            telemetry.addData("kP", robot.kp);
            telemetry.addData("kI", robot.ki);
            telemetry.addData("kD", robot.kd);
            telemetry.addData("P error", robot.error);
            telemetry.addData("I error", robot.integralSum);
            telemetry.addData("D error", robot.derivative);
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
