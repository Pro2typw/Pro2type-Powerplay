package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Just Parking")
public class Auton extends LinearOpMode {
    Robot r = new Robot();
    State state = State.START;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private SleeveDetection.ParkingPosition positon;
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (opModeInInit()) {
            sleep(100);
            positon = sleeveDetection.getPosition();
            telemetry.addData("COLOR", positon);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("COLOR", positon);
            telemetry.update();
            if (state == State.START) {
                r.fl.setPower(-1);
                r.fr.setPower(-1);
                r.bl.setPower(-1);
                r.br.setPower(-1);
                sleep(1150);
                    if (positon == SleeveDetection.ParkingPosition.LEFT) state = State.NEXT1;
                    if (positon == SleeveDetection.ParkingPosition.CENTER) state = State.NEXT2;
                    if (positon == SleeveDetection.ParkingPosition.RIGHT) state = State.NEXT3;

            } else if (state == State.NEXT1) {
                r.fl.setPower(0);
                r.fr.setPower(0);
                r.br.setPower(0);
                r.bl.setPower(0);
                sleep(1000);

                r.fl.setPower(1);
                r.fr.setPower(-1);
                r.br.setPower(1);
                r.bl.setPower(-1);
                sleep(800);

                state = State.JEJ;
            } else if (state == State.NEXT2){
                r.fl.setPower(0);
                r.fr.setPower(0);
                r.bl.setPower(0);
                r.br.setPower(0);
            } else if (state == State.NEXT3){
                r.fl.setPower(0);
                r.fr.setPower(0);
                r.br.setPower(0);
                r.bl.setPower(0);
                sleep(1000);

                r.fl.setPower(.5);
                r.fr.setPower(-.5);
                r.br.setPower(-.5);
                r.bl.setPower(.5);
                sleep(500);

                r.fl.setPower(-1);
                r.fr.setPower(1);
                r.br.setPower(-1);
                r.bl.setPower(1);
                sleep(800);
                state = State.JEJ;
            } else if (state == State.JEJ){
                r.fl.setPower(0);
                r.fr.setPower(0);
                r.bl.setPower(0);
                r.br.setPower(0);
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