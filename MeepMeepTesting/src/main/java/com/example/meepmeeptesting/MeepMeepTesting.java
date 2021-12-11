package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
        // System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(640)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(20, 20, Math.toRadians(60), Math.toRadians(60), 17.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(-90)))
                                .UNSTABLE_addTemporalMarkerOffset(1, () ->
                                        System.out.println("slide up")
                                )
                                .strafeTo(new Vector2d(5.5, -31.5))
                                .addTemporalMarker(() ->
                                        System.out.println("pivot on")
                                )
                                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                                    System.out.println("pivot off");
                                    System.out.println("slide off");
                                })
                                .waitSeconds(2)
                                .strafeTo(new Vector2d(5.5, -6))
                                .turn(Math.toRadians(90))
                                .strafeTo(new Vector2d(-59.5, -6))
                                .strafeTo(new Vector2d(-59.5, -35))
                                .build()
                )
                .start();
    }
}