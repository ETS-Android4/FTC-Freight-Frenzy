package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is our blue-side autonomous routine.
 */
@Config
@Autonomous(group = "drive")
public class BlueAltAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor carouselDrive;
    private DcMotor slideDrive;
    private DcMotor intakeDrive;
    private Servo pivotServo;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carouselDrive = hardwareMap.get(DcMotor.class, "carousel");
        slideDrive = hardwareMap.get(DcMotor.class, "slide");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake");
        pivotServo = hardwareMap.get(Servo.class, "pivot");

        TrajectorySequence t = drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    slideDrive.setPower(0.4)
                )
                .strafeTo(new Vector2d(5.5, 16.5))
                .addTemporalMarker(() ->
                    pivotServo.setPosition(0.8)
                )
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    pivotServo.setPosition(0.35);
                    slideDrive.setPower(0);
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(5.5, 67, 0))
                .forward(32.5)
                .build();

        waitForStart();

        drive.followTrajectorySequence(t);
    }
}