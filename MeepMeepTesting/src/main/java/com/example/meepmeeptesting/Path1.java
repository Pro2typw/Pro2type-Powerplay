package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Path1 {
    public static void main(String[] args) {
        double HighPoleYCoord = -8.75;

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15.5)
                .setConstraints(30, 30, 30, 30, 15.5)
                .setDimensions(15, 15.5)
                .setDriveTrainType(DriveTrainType.MECANUM)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32.75, -61.25, Math.toRadians(180)))
                                .splineTo(new Vector2d(12, -48), Math.toRadians(90))
                                .strafeTo(new Vector2d(12, -20.75))
                                .splineTo(new Vector2d(24, HighPoleYCoord), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(54.35, HighPoleYCoord))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(24, HighPoleYCoord))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(54.35, HighPoleYCoord))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(24, HighPoleYCoord))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(54.35, HighPoleYCoord))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(24, HighPoleYCoord))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(54.35, HighPoleYCoord))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(24, HighPoleYCoord))
                                .waitSeconds(0.5)
                                .splineTo(new Vector2d(48, -12), Math.toRadians(340))
                                .splineTo(new Vector2d(60, -24), Math.toRadians(270))
                                .splineTo(new Vector2d(45.5, -36), Math.toRadians(180))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
