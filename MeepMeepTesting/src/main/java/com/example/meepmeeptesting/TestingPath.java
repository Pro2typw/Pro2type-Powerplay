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

                                .back(47)
                                .turn(Math.toRadians(-45))
                                .lineTo(new Vector2d(-31, -7))
                                .lineTo(new Vector2d(-36, -12))
                                .turn(Math.toRadians(-45))
                                .lineTo(new Vector2d(-60, -12))
                                .lineTo(new Vector2d(-36, -12))
                                .turn(Math.toRadians(45))
                                .lineTo(new Vector2d(-31, -7))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}