package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Path3 {
    public static void main(String[] args) {
        double HighPoleYCoord = -8.75;
        final boolean FiveCone = true;
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15.5)
                .setConstraints(30, 30, 30, 30, 15.5)
                .setDimensions(16, 17)
                .setDriveTrainType(DriveTrainType.MECANUM)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32.75, -61.25, Math.toRadians(90)))
                                .lineTo(new Vector2d(36, -40))
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(23.5, -12, Math.toRadians(-90)))
                                .waitSeconds(3)
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(58, -12))

                                .lineToLinearHeading(new Pose2d(23.5, -12, Math.toRadians(-90)))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(58, -12, Math.toRadians(0)))
                                .waitSeconds(1.5)

                                .lineTo(new Vector2d(36, -12))







//                                .splineTo(new Vector2d(12, -48), Math.toRadians(90))
//                                .strafeTo(new Vector2d(12, -20.75))
//                                .splineTo(new Vector2d(24, HighPoleYCoord), Math.toRadians(0))
//                                .waitSeconds(0.5)
//                                .lineTo(new Vector2d(54.35, HighPoleYCoord))
//                                .turn(Math.toRadians(180))
//                                .waitSeconds(0.5)
//                                .lineTo(new Vector2d(24, HighPoleYCoord))
//                                .turn(Math.toRadians(180))
//                                .waitSeconds(0.5)
//                                .lineTo(new Vector2d(54.35, HighPoleYCoord))
//                                .turn(Math.toRadians(180))
//                                .waitSeconds(0.5)
//                                .lineTo(new Vector2d(24, HighPoleYCoord))
//                                .turn(Math.toRadians(180))
//                                .waitSeconds(0.5)
//                                .lineTo(new Vector2d(54.35, HighPoleYCoord))
//                                .turn(Math.toRadians(180))
//                                .waitSeconds(0.5)
//                                .lineTo(new Vector2d(24, HighPoleYCoord))
//                                .turn(Math.toRadians(180))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

// Pole to Cone to Pole
//.waitSeconds(0.5)
//.lineTo(new Vector2d(54.35, HighPoleYCoord))
//.turn(Math.toRadians(180))
//.waitSeconds(0.5)
//.lineTo(new Vector2d(24, HighPoleYCoord))