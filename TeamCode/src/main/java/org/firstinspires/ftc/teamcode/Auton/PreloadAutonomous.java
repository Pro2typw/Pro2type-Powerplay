package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Constants;
import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Left 0+1 Autonomous", group = "Comp Autonomous")
public class PreloadAutonomous extends LinearOpMode {
    final int NUM_CONES = 2;

    private MecanumDrive drive;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private SleeveDetection.ParkingPosition position;
    private Robot r = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap);

        r.linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.linkl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.linkr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(270));
        TrajectorySequence gotoPole = drive.trajectorySequenceBuilder(startPose)
                .back(57)
                .forward(10)
                .addDisplacementMarker(() -> {
                    r.linkl.setTargetPosition(Constants.LINKAGE_HIGH+5);
                    r.linkr.setTargetPosition(Constants.LINKAGE_HIGH+5);
                    r.linkl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkl.setPower(1);
                    r.linkr.setPower(1);
                })
                .turn(Math.toRadians(-45))
                .waitSeconds(1)
                .lineTo(new Vector2d(-34, -6)) // -34, -6; -34.5, -6.5
                .waitSeconds(1)
                .addTemporalMarker(5, () -> {
                    r.deploy();
                    r.adjust(6.5);
                })
                .addTemporalMarker(7, () -> {
                    r.clawPosition(true);
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(-36, -12))
                .addTemporalMarker(9, () -> {
                    r.clawPosition(false);
                    r.linkl.setTargetPosition(0);
                    r.linkr.setTargetPosition(0);
                    r.linkl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkl.setPower(.45);
                    r.linkr.setPower(.45);
                })
                .turn(Math.toRadians(-45))
                .build();


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


        while(opModeInInit()) {
            position = sleeveDetection.getPosition();
            telemetry.addData("Sleeve", position);
            telemetry.update();
        }
        waitForStart();

        position = sleeveDetection.getPosition();
        telemetry.addData("Sleeve", position);
        telemetry.update();

        r.clawPosition(false);

        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequence(gotoPole);
        switch (position) {
            case LEFT:
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(gotoPole.end())
                                .lineTo(new Vector2d(-36, -39))
                                .lineTo(new Vector2d(-60, -39))
                                .build()
                );
                break;
            case RIGHT:
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(gotoPole.end())
                                .lineTo(new Vector2d(-12, -12))
                                .lineTo(new Vector2d(-12, -40))

                                .build()
                );
        }
        while(opModeIsActive()) {
            drive.update();
        }
    }
}
