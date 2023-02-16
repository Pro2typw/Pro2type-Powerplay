package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.Auton.LeftAuton2.PreloadStates.INIT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ConstantsAndStuff.Robot;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Left Autonomous 2", group = "Test Autonomous")
public class LeftAuton2 extends LinearOpMode {
    final int NUM_CONES = 2;

    private MecanumDrive drive;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private SleeveDetection.ParkingPosition position;
    private Robot r = new Robot();
    public enum PreloadStates {
        INIT,
        GO_TO_POLE,
        WAIT_TILL_DONE,
        SET_BASE_POSITION,
        SET_DR4B_POSITION,
        SET_CLAW_POSITION
    }
    public PreloadStates states = INIT;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);
        r.init(hardwareMap, telemetry);
//        r.clawL.setPosition();

        Pose2d startPose = new Pose2d(-32.75, -61.25, Math.toRadians(270));
        TrajectorySequence gotoPreload = drive.trajectorySequenceBuilder(startPose)
                .back(52)
                .turn(Math.toRadians(-45))
                .lineTo(new Vector2d(-31, -7))
                //TODO: Add code to drop preload cone at high pole
//                .waitSeconds(2)
//                .lineTo(new Vector2d(-36, -12))
//                .turn(Math.toRadians(-45))
                .build();

        waitForStart();

        drive.followTrajectorySequence(gotoPreload);

//        for(int i = 0; i < NUM_CONES - 1; i++) {
//            drive.followTrajectorySequence(scoreConeCycle);
//            if((i == NUM_CONES - 2 && position != SleeveDetection.ParkingPosition.RIGHT) || i != NUM_CONES - 2) {
//                drive.followTrajectorySequence(turnRight45Deg);
//            }
//        }
//
//        switch (position) {
//            case LEFT:
//                drive.followTrajectorySequence(signalLeft);
//                break;
//            case CENTER:
////                drive.followTrajectorySequence(turnRight45Deg);
//                break;
//            default:
//                drive.followTrajectorySequence(signalRight);
//                break;
//        }

        while(opModeIsActive()) {
            switch (states) {
                case INIT:

            }
            drive.update();
        }
    }
}
