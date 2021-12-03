package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Drivetrain Straight Test OpMode", group="Linear Opmode")
public class DrivetrainStraightTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRear");

        /*
        All drives except the left rear drive are somehow hooked up in reverse, so we set their
        direction to reverse to account for it.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            leftFrontDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            leftRearDrive.setPower(0.5);
            rightRearDrive.setPower(0.5);
        }
    }
}
