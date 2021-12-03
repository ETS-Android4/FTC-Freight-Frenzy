package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class BlueAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;
    private DcMotor carouselDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRear");
        carouselDrive = hardwareMap.get(DcMotor.class, "carousel");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        //drive.turn(Math.toRadians(7.25));

        runtime.reset();

        while (opModeIsActive()) {
            /*if (runtime.milliseconds() > 500 && runtime.milliseconds() < 2000) {
                leftFrontDrive.setPower(0.85);
                rightFrontDrive.setPower(0.5);
                leftRearDrive.setPower(0.5);
                rightRearDrive.setPower(0.5);
            } else {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightRearDrive.setPower(0);
            } */
            if (runtime.milliseconds() > 500 && runtime.milliseconds() < 700) {
                leftFrontDrive.setPower(-0.85);
                rightFrontDrive.setPower(0.5);
                leftRearDrive.setPower(0.5);
                rightRearDrive.setPower(-0.5);
                carouselDrive.setPower(0);
            } else if (runtime.milliseconds() > 1200 && runtime.milliseconds() < 1400) {
                leftFrontDrive.setPower(0.6);
                rightFrontDrive.setPower(0.2);
                leftRearDrive.setPower(0.2);
                rightRearDrive.setPower(0.2);
                carouselDrive.setPower(0);
            } else if (runtime.milliseconds() > 1900 && runtime.milliseconds() < 5900) {
                carouselDrive.setPower(0.4);
            } /*else if (runtime.milliseconds() > 6400 && runtime.milliseconds() < 6850) {
                leftFrontDrive.setPower(-1);
                rightFrontDrive.setPower(0.2);
                leftRearDrive.setPower(0.7);
                rightRearDrive.setPower(-0.2);
                carouselDrive.setPower(0);
            } else if (runtime.milliseconds() > 7350 && runtime.milliseconds() < 8650) {
                leftFrontDrive.setPower(-0.85);
                rightFrontDrive.setPower(-0.5);
                leftRearDrive.setPower(-0.5);
                rightRearDrive.setPower(-0.5);
                carouselDrive.setPower(0);
            } */else if (runtime.milliseconds() > 6400 && runtime.milliseconds() < 6800) {
                leftFrontDrive.setPower(-0.85);
                rightFrontDrive.setPower(0.5);
                leftRearDrive.setPower(0.5);
                rightRearDrive.setPower(-0.5);
                carouselDrive.setPower(0);
            } else {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightRearDrive.setPower(0);
                carouselDrive.setPower(0);
            }
        }
    }
}