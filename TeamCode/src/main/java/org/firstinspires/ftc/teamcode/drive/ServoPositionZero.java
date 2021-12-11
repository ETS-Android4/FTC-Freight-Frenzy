package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Position Zero", group="Linear Opmode")
public class ServoPositionZero extends LinearOpMode {
    private Servo pivotServo;

    @Override
    public void runOpMode() {
        pivotServo = hardwareMap.get(Servo.class, "pivot");
        waitForStart();
        while (opModeIsActive()) {
            pivotServo.setPosition(0);
        }
    }
}
