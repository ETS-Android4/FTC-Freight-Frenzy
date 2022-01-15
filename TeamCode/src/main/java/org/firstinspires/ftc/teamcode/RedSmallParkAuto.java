package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.BarcodeDeterminer.BarcodeDeterminationPipeline;
import org.firstinspires.ftc.teamcode.BarcodeDeterminer.BarcodeDeterminationPipeline.BarcodePosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/*
 * This is our red-side autonomous routine.
 */
@Config
@Autonomous(group = "drive")
public class RedSmallParkAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor carouselDrive;
    private DcMotor slideDrive;
    private DcMotor intakeDrive;
    private Servo pivotServo;

    OpenCvInternalCamera phoneCam;
    BarcodeDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carouselDrive = hardwareMap.get(DcMotor.class, "carousel");
        slideDrive = hardwareMap.get(DcMotor.class, "slide");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake");
        pivotServo = hardwareMap.get(Servo.class, "pivot");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BarcodeDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        BarcodePosition result = pipeline.getAnalysis();

        telemetry.addData("Analysis", result);
        telemetry.update();

        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();

        Pose2d p1 = new Pose2d(-35.5, -62.125, Math.toRadians(90));
        TrajectorySequence t1;
        Pose2d p2;
        if (result == BarcodePosition.LEFT) {
            t1 = drive.trajectorySequenceBuilder(p1)
                    .addTemporalMarker(() ->
                            slideDrive.setPower(0.5)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(2, () ->
                            slideDrive.setPower(-0.4)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(2.3, () ->
                            slideDrive.setPower(0)
                    )
                    .lineTo(new Vector2d(-26.5, -26.5))
                    .build();
            p2 = new Pose2d(-26.5, -26.5, Math.toRadians(90));
        } else if (result == BarcodePosition.CENTER) {
            t1 = drive.trajectorySequenceBuilder(p1)
                    .addTemporalMarker(() ->
                            slideDrive.setPower(0.5)
                    )
                    .lineTo(new Vector2d(-31, -26.5))
                    .build();
            p2 = new Pose2d(-31, -26.5, Math.toRadians(90));
        } else {
            t1 = drive.trajectorySequenceBuilder(p1)
                    .addTemporalMarker(() ->
                            slideDrive.setPower(0.5)
                    )
                    .lineTo(new Vector2d(-26.5, -26.5))
                    .build();
            p2 = new Pose2d(-26.5, -26.5, Math.toRadians(90));
        }

        TrajectorySequence t2 = drive.trajectorySequenceBuilder(p2)
                .addTemporalMarker(() ->
                        pivotServo.setPosition(0.07)
                )
                .UNSTABLE_addTemporalMarkerOffset(2, () ->
                        pivotServo.setPosition(0.52)
                )
                .UNSTABLE_addTemporalMarkerOffset(2.5, () ->
                        slideDrive.setPower(-0.4)
                )
                .UNSTABLE_addTemporalMarkerOffset(2.8, () ->
                        slideDrive.setPower(0)
                )
                .waitSeconds(2)
                .lineTo(new Vector2d(-58, -26.5))
                .turn(Math.toRadians(180))
                .lineTo(new Vector2d(-66, -26.5)) // contact at -64.25
                .setVelConstraint(new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(-66, -56)) // contact at -53.675?
                .resetVelConstraint()
                .addTemporalMarker(() ->
                        carouselDrive.setPower(-0.6)
                )
                .UNSTABLE_addTemporalMarkerOffset(3, () ->
                        carouselDrive.setPower(0)
                )
                .waitSeconds(3.2)
                .lineTo(new Vector2d(-66, -33.5))
                .build();

        pivotServo.setPosition(0.52);
        drive.setPoseEstimate(p1);
        drive.followTrajectorySequence(t1);
        drive.setPoseEstimate(p2);
        drive.followTrajectorySequence(t2);
    }
}