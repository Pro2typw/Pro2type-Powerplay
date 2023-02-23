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

@Autonomous(name = "Left Autonomous", group = "Autonomous")
public class LeftAuton extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(270));
        TrajectorySequence gotoPole = drive.trajectorySequenceBuilder(startPose)
                .back(57)
                .forward(10)
                .addDisplacementMarker(() -> {
                    r.linkl.setTargetPosition(Constants.LINKAGE_HIGH + 15);
                    r.linkr.setTargetPosition(Constants.LINKAGE_HIGH + 15);
                    r.linkl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkl.setPower(.35);
                    r.linkr.setPower(.35);
                })
                .turn(Math.toRadians(-45))
                .waitSeconds(1)
                .lineTo(new Vector2d(-35, -6.5)) // -34, -6
                .waitSeconds(1)
                .addTemporalMarker(6, () -> {
                    r.deploy();
                })
                .addTemporalMarker(8, () -> {
                    r.clawPosition(true);
                })
                .waitSeconds(3)
                .lineTo(new Vector2d(-36, -12))
                .addTemporalMarker(9, () -> {
                    r.clawPosition(false);
                    r.linkl.setTargetPosition(0);
                    r.linkr.setTargetPosition(0);
                    r.linkl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkl.setPower(.25);
                    r.linkr.setPower(.25);
                })
                .turn(Math.toRadians(135))
                .waitSeconds(1)
                .addTemporalMarker(10, () -> {
                    r.deploy();
                    r.adjust(.05);
                    r.clawPosition(true);
                })
                .lineTo(new Vector2d(-56, -12))
                .addTemporalMarker(14, () -> {
                    r.clawPosition(false);
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(-59, -12))


                .waitSeconds(1)
                .lineTo(new Vector2d(-36, -12))
                .addTemporalMarker(16, () -> {
                    r.clawPosition(false);
                    r.linkl.setTargetPosition(Constants.LINKAGE_HIGH);
                    r.linkr.setTargetPosition(Constants.LINKAGE_HIGH);
                    r.linkl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    r.linkl.setPower(.25);
                    r.linkr.setPower(.25);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(-135))
                .addTemporalMarker(18, () -> {
                    r.deploy();
                })
                .lineTo(new Vector2d(-35, -6.5))
                .addTemporalMarker(19, () -> {
                    r.clawPosition(true);
                })
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
        telemetry.addData("Sleeve", position);
        telemetry.update();

        waitForStart();

        telemetry.addData("Sleeve", position);
        telemetry.update();
        position = sleeveDetection.getPosition();

        r.clawPosition(false);

        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequence(gotoPole);

        while(opModeIsActive()) {
            drive.update();
        }
    }
}
