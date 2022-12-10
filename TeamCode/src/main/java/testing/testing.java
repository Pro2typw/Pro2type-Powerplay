package testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;

@TeleOp
public class testing extends LinearOpMode {
    Robot r = new Robot();
    public void runOpMode() throws InterruptedException {

        Robot r = new Robot();

        r.init(hardwareMap, telemetry);

        waitForStart();
        r.telemetry();
        if (isStopRequested()) return;
    }
}
