package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
        // System.setProperty("sun.java2d.opengl", "true");

        // at most 10.5 inch carousel
        // 2 inches from center of robot to chute
        // 5.5 inches from center of robot to center of carousel
        // carousel 5.85 inches from corner
        // carousel contact 11.32 inches from wall
        MeepMeep mm = new MeepMeep(640)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .setConstraints(45, 30, Math.toRadians(90), Math.toRadians(90), 16.8)
                .setBotDimensions(12.5, 16.75)
                .followTrajectorySequence(drive -> {
                    Pose2d p1 = new Pose2d(12, -62.125, Math.toRadians(90));
                    TrajectorySequence t1 = drive.trajectorySequenceBuilder(p1)
                            .lineTo(new Vector2d(9, -26.5))
                            .waitSeconds(2)
                            .back(31.5)
                            .turn(Math.toRadians(-90))
                            .strafeRight(8)
                            .lineTo(new Vector2d(40, -66))
                            .build();
                    return t1;
                })
                .start();
    }
}