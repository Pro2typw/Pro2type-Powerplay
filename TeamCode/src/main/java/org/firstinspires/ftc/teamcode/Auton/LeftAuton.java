package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Left Autonomous", group = "Autonomous")
public class LeftAuton extends LinearOpMode {
    final int NUM_CONES = 5;

    private MecanumDrive drive = new MecanumDrive(hardwareMap);
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private SleeveDetection.ParkingPosition positon;

    Pose2d startPose = new Pose2d(-32.75, -61.25, Math.toRadians(90));
    TrajectorySequence scorePreloadCone = drive.trajectorySequenceBuilder(startPose)
            .lineTo(new Vector2d(-36, -12))
            .turn(Math.toRadians(-45))
            .lineTo(new Vector2d(-31, -7))
            //TODO: Add code to drop preload cone at high pole
            .waitSeconds(2)
            .lineTo(new Vector2d(-36, -12))
            .turn(Math.toRadians(-45))
            .build();

    TrajectorySequence scoreConeCycle = drive.trajectorySequenceBuilder(scorePreloadCone.end())
            .lineTo(new Vector2d(-60, -12))
            .waitSeconds(1)
            .lineTo(new Vector2d(-36, -12))
            .turn(Math.toRadians(45))
            .lineTo(new Vector2d(-31, -7))
            //TODO: Add code for dropping cone at high pole
            .waitSeconds(2)
            .lineTo(new Vector2d(-36, -12))
            .build();

    TrajectorySequence turnRight45Deg = drive.trajectorySequenceBuilder(scoreConeCycle.end())
            .turn(Math.toRadians(-45))
            .build();

    // Parking Positions
    TrajectorySequence signalLeft = drive.trajectorySequenceBuilder(turnRight45Deg.end())
            .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
            .splineTo(new Vector2d(-12, -24), Math.toRadians(270))
            .splineTo(new Vector2d(-12, -36), Math.toRadians(270))
            .build();

    TrajectorySequence signalRight = drive.trajectorySequenceBuilder(turnRight45Deg.end())
            .turn(Math.toRadians(135))
            .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
            .splineTo(new Vector2d(-60, -24), Math.toRadians(270))
            .splineTo(new Vector2d(-60, -36), Math.toRadians(270))
            .build();
    @Override
    public void runOpMode() throws InterruptedException {
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

        positon = sleeveDetection.getPosition();
        telemetry.addData("Sleeve", positon);

        drive.setPoseEstimate(startPose);

        waitForStart();

        drive.followTrajectorySequence(scorePreloadCone);

        for(int i = 0; i < NUM_CONES - 1; i++) {
            drive.followTrajectorySequence(scoreConeCycle);
            if((i == NUM_CONES - 2 && positon != SleeveDetection.ParkingPosition.RIGHT) || i != NUM_CONES - 2) {
                drive.followTrajectorySequence(turnRight45Deg);
            }
        }

        switch (positon) {
            case LEFT:
                drive.followTrajectorySequence(signalLeft);
                break;
            case CENTER:
                drive.followTrajectorySequence(turnRight45Deg);
                break;

            default:
                drive.followTrajectorySequence(signalRight);
                break;
        }

        while(opModeIsActive()) {
            drive.update();
        }
    }
}
