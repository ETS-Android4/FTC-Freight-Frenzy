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
                    Pose2d p1 = new Pose2d(12, 62.125, Math.toRadians(-90));
                    TrajectorySequence t1 = drive.trajectorySequenceBuilder(p1)
                            .lineTo(new Vector2d(4.5, 24.5))
                            .build();
                    Pose2d p2 = new Pose2d(4.5, 24.5, Math.toRadians(-90));
                    TrajectorySequence t2 = drive.trajectorySequenceBuilder(p2)
                            .waitSeconds(2)
                            .lineToLinearHeading(p2.plus(new Pose2d(0, 33.5, Math.toRadians(90))))
                            .strafeLeft(8)
                            .lineTo(new Vector2d(40, 66))
                            .build();
                    Pose2d p3 = new Pose2d(40, 64.25, Math.toRadians(0));
                    TrajectorySequence t3 = drive.trajectorySequenceBuilder(p3)
                            .lineTo(new Vector2d(52, 64.25))
                            .lineTo(new Vector2d(22, 64.25))
                            .splineTo(new Vector2d(-13, 40), Math.toRadians(180))
                            .waitSeconds(1)
                            .splineTo(new Vector2d(22, 64.25), Math.toRadians(0))
                            .lineTo(new Vector2d(40, 64.25))
                            .build();
                    System.out.println(t1.getDuration() + t2.getDuration() + (t3.getDuration() * 2));
                    return t1;
                })
                .start();
    }
}