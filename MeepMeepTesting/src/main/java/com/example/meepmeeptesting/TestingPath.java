package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TestingPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPos = new Pose2d(-36, -58, Math.toRadians(270));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(startPos)
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15.5)
                .setConstraints(30, 30, 30, 30, 15.5)
                .setDimensions(16, 17)
                .setDriveTrainType(DriveTrainType.MECANUM)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .back(57)
                                .forward(10)
//                                .addDisplacementMarker(() -> {
//                                    r.linkl.setTargetPosition(Constants.LINKAGE_HIGH + 25);
//                                    r.linkr.setTargetPosition(Constants.LINKAGE_HIGH + 25);
//                                    r.linkl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    r.linkr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    r.linkl.setPower(.35);
//                                    r.linkr.setPower(.35);
//                                })
                                .turn(Math.toRadians(-45))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-32.5, -5)) // -34, -6; -34.5, -6.5
                                .waitSeconds(1)
//                                .addTemporalMarker(6, () -> {
//                                    r.deploy();
//                                })
//                                .addTemporalMarker(8, () -> {
//                                    r.clawPosition(true);
//                                })
                                .waitSeconds(3)
                                .lineTo(new Vector2d(-36, -12))
//                                .addTemporalMarker(9, () -> {
//                                    r.clawPosition(false);
//                                    r.linkl.setTargetPosition(0);
//                                    r.linkr.setTargetPosition(0);
//                                    r.linkl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    r.linkr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    r.linkl.setPower(.25);
//                                    r.linkr.setPower(.25);
//                                })
                                .turn(Math.toRadians(-45))
                                .waitSeconds(1)
//                                .addTemporalMarker(12, () -> {
//                                    r.intakePrep();
//                                    r.linkl.setTargetPosition(Constants.LINKAGE_DOWN - 10);
//                                    r.linkr.setTargetPosition(Constants.LINKAGE_DOWN - 10);
//
//                                    r.clawPosition(true);
//                                })
                                .lineTo(new Vector2d(-64, -12))
                                .waitSeconds(1)
//                                .addTemporalMarker(15, () -> {
//                                    r.clawPosition(false);
//                                    r.hold();
//                                })
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-36, -12))
//                                .addTemporalMarker(18, () -> {
//                                    r.clawPosition(false);
//                                    r.linkl.setTargetPosition(Constants.LINKAGE_HIGH);
//                                    r.linkr.setTargetPosition(Constants.LINKAGE_HIGH);
//                                    r.linkl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    r.linkr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    r.linkl.setPower(.25);
//                                    r.linkr.setPower(.25);
//                                })
                                .waitSeconds(1)
                                .turn(Math.toRadians(45))
//                                .addTemporalMarker(20, () -> {
//                                    r.deploy();
//                                })
                                .lineTo(new Vector2d(-34, -6))
//                                .addTemporalMarker(21, () -> {
//                                    r.clawPosition(true);
//                                })
                                .waitSeconds(3)
                                .lineTo(new Vector2d(-36, -12))
//                                .addTemporalMarker(23, () -> {
//                                    r.clawPosition(false);
//                                    r.linkl.setTargetPosition(0);
//                                    r.linkr.setTargetPosition(0);
//                                    r.linkl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    r.linkr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    r.linkl.setPower(.25);
//                                    r.linkr.setPower(.25);
//                                })
                                .turn(Math.toRadians(-45))


                                .lineTo(new Vector2d(-36, -35))
                                .lineTo(new Vector2d(-56, -35))

                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}