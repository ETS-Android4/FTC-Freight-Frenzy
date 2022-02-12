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
                    Pose2d p1 = new Pose2d(-35.5, 62.125, Math.toRadians(-90));
                    TrajectorySequence t1 = drive.trajectorySequenceBuilder(p1)
                            .lineTo(new Vector2d(-28, 24.5))
                            .waitSeconds(2)
                            .lineToLinearHeading(new Pose2d(-66, 24.5, Math.toRadians(90))) // contact at -64.25
                            .build();

                    Pose2d p2 = new Pose2d(-64.25, 24.5, Math.toRadians(90));
                    TrajectorySequence t2 = drive.trajectorySequenceBuilder(p2)
                            .setVelConstraint(new MecanumVelocityConstraint(20, 16.4))
                            .lineTo(new Vector2d(-64.25, 56)) // contact at 53.675?
                            .resetVelConstraint()
                            .waitSeconds(3.2)
                            .lineToLinearHeading(new Pose2d(-30, 56, Math.toRadians(0)))
                            .lineTo(new Vector2d(-30, 68)) // contact at 64.25
                            .lineTo(new Vector2d(40, 68))
                            .build();

                    Pose2d p3 = new Pose2d(40, 64.25, Math.toRadians(0));
                    TrajectorySequence t3 = drive.trajectorySequenceBuilder(p3)
                            .lineTo(new Vector2d(62, 64.25))
                            .build();
                    System.out.println(t1.getDuration() + t2.getDuration() + (t3.getDuration() * 2));
                    return t2;
                })
                .start();
    }
}