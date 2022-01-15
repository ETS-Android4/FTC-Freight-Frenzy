package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Position Half", group="Linear Opmode")
public class ServoPositionHalf extends LinearOpMode {
    private Servo pivotServo;

    @Override
    public void runOpMode() {
        pivotServo = hardwareMap.get(Servo.class, "pivot");
        waitForStart();
        while (opModeIsActive()) {
            pivotServo.setPosition(0.5);
        }
    }
}
