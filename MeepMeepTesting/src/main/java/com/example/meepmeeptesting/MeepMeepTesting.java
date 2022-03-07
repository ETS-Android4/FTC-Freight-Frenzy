package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
        // System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(640)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .setConstraints(60, 45, Math.toRadians(120), Math.toRadians(90), 16.8)
                .setBotDimensions(12.5, 16.75)
                .followTrajectorySequence(drive -> {
                    Pose2d p3 = new Pose2d(-64.25, 56, Math.toRadians(90));
                    TrajectorySequence t3 = drive.trajectorySequenceBuilder(p3)
                            .lineTo(new Vector2d(-60, 36))
                            .build();
                    return t3;
                })
                .start();
    }
}