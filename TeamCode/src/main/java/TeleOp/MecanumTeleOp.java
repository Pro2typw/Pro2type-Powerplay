package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;

@TeleOp (name = "teleopRed")

public class MecanumTeleOp extends LinearOpMode {

    Robot r = new Robot();

    boolean deployed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        r.init(hardwareMap);

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
                target = 20;
                adjustment = 0;
            }
            if (gamepad2.dpad_up) {
                target = 180;
                adjustment = 0;
            }
            if (gamepad1.dpad_left) {
                target = 90;
                adjustment = 0;
            }

            adjustment += (int) (gamepad2.left_stick_y * 4);

            r.linkr.setPower(r.PIDController(target + adjustment, r.getPos(r.linkr)));
            r.linkl.setPower(r.PIDController(target + adjustment, r.getPos(r.linkl)));

        }
    }
}