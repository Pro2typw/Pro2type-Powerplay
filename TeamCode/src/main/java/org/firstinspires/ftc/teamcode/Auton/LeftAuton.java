package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection.ParkingPosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Left Autonomous")
public class LeftAuton extends LinearOpMode {
    final int NUM_CONES = 5;
    //    private Robot r = new Robot();

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;
    ParkingPosition positon;



    @Override
    public void runOpMode() throws InterruptedException {
//        r.init(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(32.75, -61.25, Math.toRadians(90));

        TrajectorySequence preSetup = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(36, -46))
                .addTemporalMarker(1, () -> {
                    positon = sleeveDetection.getPosition();
                })
                .splineTo(new Vector2d(36, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(24, -11), Math.toRadians(180))
                .addDisplacementMarker(() -> {

                })
                .build();

        TrajectorySequence scoreCone = drive.trajectorySequenceBuilder(preSetup.end())
                .lineToLinearHeading(new Pose2d(56, -12, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    // Get Cone
//                r.intake();

                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    // Place Cone
//                r.state = Robot.StateDR4B.TOP;
//                r.DR4BState();

                })
                .waitSeconds(1)
                .build();

        TrajectorySequence signalLeft = drive.trajectorySequenceBuilder(scoreCone.end())
                .lineTo(new Vector2d(12, -11))
                .lineTo(new Vector2d(12, -36))
                .build();

        TrajectorySequence signalCenter = drive.trajectorySequenceBuilder(scoreCone.end())
                .lineTo(new Vector2d(36, -11))
                .build();

        TrajectorySequence signalRight = drive.trajectorySequenceBuilder(scoreCone.end())
                .lineTo(new Vector2d(58, -11))
                .lineTo(new Vector2d(58, -36))
                .build();

        // Camera Code
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
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

        drive.setPoseEstimate(startPose);

        telemetry.addData("Sleeve", positon);

        waitForStart();

        positon = sleeveDetection.getPosition();


        drive.followTrajectorySequence(preSetup);

        for(int i = 0; i < NUM_CONES - 1; i++) drive.followTrajectorySequence(scoreCone);

        switch (positon) {
            case LEFT:
                drive.followTrajectorySequence(signalLeft);
                break;
            case CENTER:
                drive.followTrajectorySequence(signalCenter);
                break;

            default:
                drive.followTrajectorySequence(signalRight);
                break;
        }

        while (opModeIsActive()) {
            drive.update();
        }
    }
}
