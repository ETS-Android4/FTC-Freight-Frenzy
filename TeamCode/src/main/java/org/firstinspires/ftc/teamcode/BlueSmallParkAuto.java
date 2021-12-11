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
public class BlueSmallParkAuto extends LinearOpMode {
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

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 63, Math.toRadians(90)))
                .strafeTo(new Vector2d(-64, 46))
                .strafeLeft(4)
                .forward(10)
                .addTemporalMarker(() ->
                        carouselDrive.setPower(0.6)
                )
                .UNSTABLE_addTemporalMarkerOffset(3, () ->
                        carouselDrive.setPower(0)
                )
                .waitSeconds(3.2)
                .build();
        TrajectorySequence t2 = drive.trajectorySequenceBuilder(new Pose2d(-65.15, 53, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-25, 50, 0))
                .strafeTo(new Vector2d(-25, 69))
                .build();
        TrajectorySequence t3 = drive.trajectorySequenceBuilder(new Pose2d(-25, 65.15, 0))
                .UNSTABLE_addTemporalMarkerOffset(1, () ->
                        slideDrive.setPower(0.4)
                )
                .strafeTo(new Vector2d(-4, 39))
                .addTemporalMarker(() ->
                        pivotServo.setPosition(0.8)
                )
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    pivotServo.setPosition(0.35);
                    slideDrive.setPower(0);
                })
                .waitSeconds(2)
                .strafeTo(new Vector2d(-59.5, 39))
                .strafeTo(new Vector2d(-59.5, 35))
                .build();

        waitForStart();

        drive.followTrajectorySequence(t1);
        drive.followTrajectorySequence(t2);
        drive.followTrajectorySequence(t3);
    }
}