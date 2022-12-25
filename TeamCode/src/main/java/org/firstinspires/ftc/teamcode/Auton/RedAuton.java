package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;
import org.firstinspires.ftc.teamcode.vision.SignalDetectionPipeline;

@Autonomous
public class RedAuton extends LinearOpMode {
    Robot r = new Robot();
    State state = State.START;
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap, telemetry);



        waitForStart();

        while (opModeIsActive()) {

            if (state == State.START) {
                r.move();
                if (r.getPos(r.fr) > 20) {
                    if (SignalDetectionPipeline.val == 0); state = State.NEXT1;
                    if (SignalDetectionPipeline.val == 1); state = State.NEXT2;
                    if (SignalDetectionPipeline.val == 2); state = State.NEXT3;
                }
            } else if (state == State.NEXT1) {
                r.strafeR();
                if (r.getPos(r.fl) > 40) state = State.JEJ;
            } else if (state == State.NEXT2){
                r.park();
            } else if (state == State.NEXT3){
                r.strafeL();
                if (r.getPos(r.fr) > 40) state = State.JEJ;
            } else if (state == State.JEJ){
                r.park();
            }
            telemetry.update();

        }
    }

    public enum State {
        START,
        NEXT1,
        NEXT2,
        NEXT3,
        JEJ
    }
}
