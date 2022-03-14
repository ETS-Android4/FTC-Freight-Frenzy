package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Red Mecanum OpMode", group="Linear Opmode")
public class RedMecanumDrive extends LinearOpMode {
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

        intakeDrive.setDirection(DcMotor.Direction.REVERSE);

        slideController = new SlidePIDController(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            final double drive = -gamepad1.left_stick_y * 0.7;
            final double strafe = gamepad1.left_stick_x * 0.71;
            final double turn = gamepad1.right_stick_x * 0.9;

            /*
            Using Mecanum wheel behaviors presented here:
            https://docs.revrobotics.com/15mm/ftc-starter-kit-mecanum-drivetrain/mecanum-wheel-setup-and-behavior
             */
            final double leftFrontPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
            final double rightFrontPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
            final double leftRearPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
            final double rightRearPower = Range.clip(drive + strafe - turn, -1.0, 1.0);

            final double carouselPower = (gamepad2.right_trigger - gamepad2.left_trigger) * 0.7;
            final double intakePower = (gamepad2.right_bumper ? 1.0 : 0.0) - (gamepad2.left_bumper ? 1.0 : 0.0);

            final double pivotPosition = 0.51 + (gamepad2.square ? 0.46 : 0.0) + (gamepad2.circle ? -0.46 : 0.0);

            if (gamepad2.dpad_down) {
                slideController.setTarget(0.1);
            } else if (gamepad2.dpad_left) {
                slideController.setTarget(2);
            } else if (gamepad2.dpad_up) {
                slideController.setTarget(3);
            } else if (gamepad2.dpad_right) {
                slideController.setTarget(4.5);
            }

            if (gamepad1.cross) {
                slideController.setPower(0);
            } else if (gamepad1.square) {
                slideController.setPower(-1);
            } else if (gamepad1.circle) {
                slideController.setPower(1);
            } else if (gamepad1.triangle) {
                slideController.resetEncoder();
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
