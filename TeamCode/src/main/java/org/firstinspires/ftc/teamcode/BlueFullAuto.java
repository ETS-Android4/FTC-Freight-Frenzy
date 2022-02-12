package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancellable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.BarcodeDeterminer.BarcodeDeterminationPipeline;
import org.firstinspires.ftc.teamcode.BarcodeDeterminer.BarcodeDeterminationPipeline.BarcodePosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(group = "drive")
public class BlueFullAuto extends LinearOpMode {
    private DcMotor carouselDrive;
    private DcMotor intakeDrive;
    private Servo pivotServo;
    private SlidePIDController slideController;

    private OpenCvInternalCamera phoneCam;
    private BarcodeDeterminationPipeline pipeline;

    private enum State {
        PRELOAD,
        CAROUSEL,
        CARGO,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancellable drive = new SampleMecanumDriveCancellable(hardwareMap);

        carouselDrive = hardwareMap.get(DcMotor.class, "carousel");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake");
        pivotServo = hardwareMap.get(Servo.class, "pivot");
        intakeDrive.setDirection(DcMotor.Direction.REVERSE);

        slideController = new SlidePIDController(hardwareMap);

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
            public void onError(int errorCode) {}
        });

        Pose2d p1 = new Pose2d(-35.5, 62.125, Math.toRadians(-90));
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(p1)
                .lineTo(new Vector2d(-30, 24.5))
                .addTemporalMarker(() ->
                        pivotServo.setPosition(0.97)
                )
                .UNSTABLE_addTemporalMarkerOffset(1.4, () ->
                        pivotServo.setPosition(0.51)
                )
                .UNSTABLE_addTemporalMarkerOffset(1.7, () ->
                        slideController.setTarget(0.1)
                )
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(-66, 24.5, Math.toRadians(90))) // contact at -64.25
                //.strafeLeft(4) // just to be safe
                .build();

        Pose2d p2 = new Pose2d(-64.25, 24.5, Math.toRadians(90));
        TrajectorySequence t2 = drive.trajectorySequenceBuilder(p2)
                .setVelConstraint(new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(-64.25, 56)) // contact at 53.675?
                .resetVelConstraint()
                .addTemporalMarker(() ->
                        carouselDrive.setPower(0.6)
                )
                .UNSTABLE_addTemporalMarkerOffset(3, () ->
                        carouselDrive.setPower(0)
                )
                .waitSeconds(3.2)
                .lineToLinearHeading(new Pose2d(-30, 56, Math.toRadians(0)))
                .lineTo(new Vector2d(-30, 68)) // contact at 64.25
                .lineTo(new Vector2d(40, 68))
                .build();

        Pose2d p3 = new Pose2d(40, 64.25, Math.toRadians(0));
        TrajectorySequence t3 = drive.trajectorySequenceBuilder(p3)
                .addTemporalMarker(() ->
                        intakeDrive.setPower(1.0)
                )
                .lineTo(new Vector2d(62, 64.25))
                .addTemporalMarker(() -> {
                    intakeDrive.setPower(-1.0);
                    slideController.setTarget(0.1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () ->
                        intakeDrive.setPower(0)
                )
                .lineTo(new Vector2d(22, 64.25))
                .addTemporalMarker(() ->
                        slideController.setTarget(4.5)
                )
                .splineTo(new Vector2d(-13, 43), Math.toRadians(180))
                .addTemporalMarker(() ->
                        pivotServo.setPosition(0.05)
                )
                .UNSTABLE_addTemporalMarkerOffset(1.4, () ->
                        pivotServo.setPosition(0.51)
                )
                .UNSTABLE_addTemporalMarkerOffset(1.5, () ->
                        slideController.setTarget(0.1)
                )
                .waitSeconds(1.5)
                .splineTo(new Vector2d(22, 64.25), Math.toRadians(0))
                .lineTo(new Vector2d(40, 64.25))
                .build();

        waitForStart();

        BarcodePosition result = pipeline.getAnalysis();

        telemetry.addData("Analysis", result);
        telemetry.update();

        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();

        pivotServo.setPosition(0.51);

        State currentState = State.PRELOAD;
        drive.setPoseEstimate(p1);
        drive.followTrajectorySequenceAsync(t1);
        if (result == BarcodePosition.LEFT) {
            slideController.setTarget(1);
        } else if (result == BarcodePosition.CENTER) {
            slideController.setTarget(2.5);
        } else if (result == BarcodePosition.RIGHT) {
            slideController.setTarget(4);
        }

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case PRELOAD:
                    if (!drive.isBusy()) {
                        currentState = State.CAROUSEL;
                        drive.setPoseEstimate(p2);
                        drive.followTrajectorySequenceAsync(t2);
                    }
                    break;
                case CAROUSEL:
                    if (!drive.isBusy()) {
                        currentState = State.CARGO;
                        drive.setPoseEstimate(p3);
                        drive.followTrajectorySequenceAsync(t3);
                    }
                    break;
                case CARGO:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();
            slideController.update();
        }
    }
}