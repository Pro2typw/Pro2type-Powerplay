package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBlue;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Old Right Autonomous", group = "Autonomous")
public class RightAuton extends LinearOpMode {
    final int NUM_CONES = 15;
    private final double ANGLE_MULTIPLIER = .99;

    private MecanumDrive drive;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private SleeveDetection.ParkingPosition positon;
    private Robot r = new Robot();

    // Param --> start
    TeleOpBlue teleOpBlue = new TeleOpBlue();
    // Param

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(32.75, -61.25, Math.toRadians(270));
        TrajectorySequence scorePreloadCone = drive.trajectorySequenceBuilder(startPose)
                .back(52)
                .turn(Math.toRadians(angleOffset(45)))
                .lineTo(new Vector2d(31, -7))
                //TODO: Add code to drop preload cone at high pole
                .waitSeconds(2)
                .lineTo(new Vector2d(36, -12))
                .turn(Math.toRadians(angleOffset(45)))
                .build();

        TrajectorySequence scoreConeCycle = drive.trajectorySequenceBuilder(scorePreloadCone.end())
                .lineTo(new Vector2d(60, -12))
                //TODO: PICKUP
                .addDisplacementMarker(() -> {
                    r.intakePrep();
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    r.open = false;
                    r.clawPosition(r.open);
                    r.hold();
                })
                .waitSeconds(.5)
                .lineTo(new Vector2d(36, -12))
                .addDisplacementMarker(() -> {
                    /* r.linkl.setTargetPosition(-400);
                    r.linkr.setTargetPosition(-400);
                    r.linkl.setPower(.1);
                    r.linkr.setPower(.1);

                     */
                })
                .turn(Math.toRadians(angleOffset(-45)))
                .lineTo(new Vector2d(31, -7))
                //TODO: Add code for dropping cone at high pole
                .addDisplacementMarker(() -> {
                    r.open = true;
                    r.clawPosition(r.open);
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    r.open = false;
                    r.clawPosition(r.open);
                    /*
                    r.linkl.setTargetPosition(0);
                    r.linkr.setTargetPosition(0);
                    r.linkl.setPower(.05);
                    r.linkr.setPower(.05);

                     */
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(36, -12))
                .build();

        TrajectorySequence turnLeft45Deg = drive.trajectorySequenceBuilder(scoreConeCycle.end())
                .turn(Math.toRadians(angleOffset(45)))
                .build();

        // Parking Positions
        TrajectorySequence signalLeft = drive.trajectorySequenceBuilder(turnLeft45Deg.end())
                .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                .splineTo(new Vector2d(12, -24), Math.toRadians(270))
                .splineTo(new Vector2d(12, -36), Math.toRadians(270))
                .build();

        TrajectorySequence signalRight = drive.trajectorySequenceBuilder(turnLeft45Deg.end())
                .turn(Math.toRadians(angleOffset(-135)))
                .splineTo(new Vector2d(48, -12), Math.toRadians(0))
                .splineTo(new Vector2d(60, -24), Math.toRadians(270))
                .splineTo(new Vector2d(60, -36), Math.toRadians(270))
                .build();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        positon = sleeveDetection.getPosition();
        telemetry.addData("Sleeve", positon);

        drive.setPoseEstimate(startPose);

        waitForStart();

        drive.followTrajectorySequence(scorePreloadCone);

        for (int i = 0; i < NUM_CONES - 1; i++) {
            drive.followTrajectorySequence(scoreConeCycle);
            if ((i == NUM_CONES - 2 && positon != SleeveDetection.ParkingPosition.RIGHT) || i != NUM_CONES - 2) {
                drive.followTrajectorySequence(turnLeft45Deg);
            }
        }

        switch (positon) {
            case LEFT:
                drive.followTrajectorySequence(signalLeft);
                break;
            case CENTER:
                drive.followTrajectorySequence(turnLeft45Deg);
                break;

            default:
                drive.followTrajectorySequence(signalRight);
                break;
        }

        while (opModeIsActive()) {
            drive.update();
        }

        /*
        Servo hardwareDevice = hardwareMap.servo.get("rightClaw");
        hardwareDevice.setPosition(Constants.rOpen);

        r.open = false;
        r.clawPosition(r.open);
        r.hold();
        r.adjustment = 0;
        r.state = Robot.StateDR4B.TOP;
        r.deploying = Robot.DeployingStateDR4B.WAIT;
         */
    }
    private double angleOffset(double angle) {
        return ANGLE_MULTIPLIER * angle;
    }
}
