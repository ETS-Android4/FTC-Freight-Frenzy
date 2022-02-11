package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BarcodeDeterminer.BarcodeDeterminationPipeline;
import org.firstinspires.ftc.teamcode.BarcodeDeterminer.BarcodeDeterminationPipeline.BarcodePosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancellable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/*
 * This is our blue-side autonomous routine.
 */
@Config
@Autonomous(group = "drive")
public class BlueAltAuto extends LinearOpMode {
    private DcMotor carouselDrive;
    private DcMotor intakeDrive;
    private Servo pivotServo;
    private SlidePIDController slideController;

    private OpenCvInternalCamera phoneCam;
    private BarcodeDeterminationPipeline pipeline;

    private DistanceSensor chuteProximitySensor;

    enum State {
        PRELOAD,
        CARGO,
        OUTTAKE,
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

        chuteProximitySensor = hardwareMap.get(DistanceSensor.class, "chuteSensor");

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

        Pose2d p1 = new Pose2d(12, 62.125, Math.toRadians(-90));
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(p1)
                .lineTo(new Vector2d(5, 24.5))
                .addTemporalMarker(() ->
                        pivotServo.setPosition(0.05)
                )
                .UNSTABLE_addTemporalMarkerOffset(2, () ->
                        pivotServo.setPosition(0.51)
                )
                .UNSTABLE_addTemporalMarkerOffset(2.5, () ->
                        slideController.setTarget(0.02)
                )
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(5, 58, Math.toRadians(0)))
                .strafeLeft(8)
                .lineTo(new Vector2d(40, 66))
                .build();

        Pose2d p2 = new Pose2d(40, 64.25, Math.toRadians(0));
        TrajectorySequence t2 = drive.trajectorySequenceBuilder(p2)
                .addTemporalMarker(() ->
                        intakeDrive.setPower(1.0)
                )
                .lineTo(new Vector2d(62, 64.25))
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
            slideController.setTarget(3);
        } else if (result == BarcodePosition.CENTER) {
            slideController.setTarget(3.7);
        } else if (result == BarcodePosition.RIGHT) {
            slideController.setTarget(4.5);
        }

        boolean isSecondCargo = false;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case PRELOAD:
                    if (!drive.isBusy()) {
                        currentState = State.CARGO;
                        drive.setPoseEstimate(p2);
                        drive.followTrajectorySequenceAsync(t2);
                    }
                    break;
                case CARGO:
                    if (!drive.isBusy()) {
                        currentState = State.OUTTAKE;
                        Pose2d p3 = drive.getPoseEstimate();
                        TrajectorySequence t3 = drive.trajectorySequenceBuilder(p3)
                                .addTemporalMarker(() -> {
                                    intakeDrive.setPower(-1.0);
                                    slideController.setTarget(1.0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1, () ->
                                        intakeDrive.setPower(0)
                                )
                                .lineTo(new Vector2d(22, 64.25))
                                .addTemporalMarker(() ->
                                        slideController.setTarget(3.7)
                                )
                                .splineTo(new Vector2d(-13, 40), Math.toRadians(180))
                                .addTemporalMarker(() ->
                                        pivotServo.setPosition(0.05)
                                )
                                .UNSTABLE_addTemporalMarkerOffset(0.9, () ->
                                        pivotServo.setPosition(0.51)
                                )
                                .UNSTABLE_addTemporalMarkerOffset(1, () ->
                                        slideController.setTarget(0.02)
                                )
                                .waitSeconds(1)
                                .splineTo(new Vector2d(22, 64.25), Math.toRadians(0))
                                .lineTo(new Vector2d(40, 64.25))
                                .build();
                        drive.followTrajectorySequenceAsync(t3);
                    } else if (chuteProximitySensor.getDistance(DistanceUnit.CM) < 7.5) {
                        drive.cancelFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                    break;
                case OUTTAKE:
                    if (!drive.isBusy()) {
                        if (!isSecondCargo) {
                            isSecondCargo = true;
                            currentState = State.CARGO;
                        } else {
                            currentState = State.IDLE;
                        }
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();
            slideController.update();

            telemetry.addData("lift height: (%.2f)", slideController.getSlidePosition());
            telemetry.addData("cargo proximity: (%.2f)", chuteProximitySensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}