package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Linear OpMode", group="Linear Opmode")
public class MecanumDrive extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;
    private DcMotor carouselDrive;
    private DcMotor intakeDrive;
    private Servo pivotServo;

    private SlidePIDController slideController;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRear");
        carouselDrive = hardwareMap.get(DcMotor.class, "carousel");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake");
        pivotServo = hardwareMap.get(Servo.class, "pivot");

        slideController = new SlidePIDController(hardwareMap);

        intakeDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            final double drive = -gamepad1.left_stick_y * 0.8;
            final double strafe = gamepad1.left_stick_x;
            final double turn = gamepad1.right_stick_x;

            /*
            Using Mecanum wheel behaviors presented here:
            https://docs.revrobotics.com/15mm/ftc-starter-kit-mecanum-drivetrain/mecanum-wheel-setup-and-behavior
             */
            final double leftFrontPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
            final double rightFrontPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
            final double leftRearPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
            final double rightRearPower = Range.clip(drive + strafe - turn, -1.0, 1.0);

            final double carouselPower = ((gamepad2.triangle ? 1.0 : 0.0) - (gamepad2.cross ? 1.0 : 0.0)) * 0.6;
            final double intakePower = (gamepad2.right_bumper ? 1.0 : 0.0) - (gamepad2.left_bumper ? 1.0 : 0.0);

            final double pivotPosition = 0.51 + (gamepad2.square ? 0.46 : 0.0) + (gamepad2.circle ? -0.46 : 0.0);

            if (gamepad2.dpad_down) {
                slideController.setTarget(0.02);
            } else if (gamepad2.dpad_left) {
                slideController.setTarget(2.75);
            } else if (gamepad2.dpad_up) {
                slideController.setTarget(3.75);
            } else if (gamepad2.dpad_right) {
                slideController.setTarget(4.7);
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftRearDrive.setPower(leftRearPower);
            rightRearDrive.setPower(rightRearPower);

            carouselDrive.setPower(carouselPower);
            intakeDrive.setPower(intakePower);

            final double slidePower = slideController.update();

            pivotServo.setPosition(pivotPosition);

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Lift position: (%.2f)", slideController.getSlidePosition());
            telemetry.addData("Lift power: (%.2f)", slidePower);
            telemetry.update();
        }
    }
}
