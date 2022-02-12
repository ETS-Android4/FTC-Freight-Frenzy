package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor Test", group="Linear Opmode")
public class SensorTest extends LinearOpMode {
    private DistanceSensor colorSensor;
    private SlidePIDController slideController;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(DistanceSensor.class, "chuteSensor");
        slideController = new SlidePIDController(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance: ", colorSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("lift height: ", slideController.getSlidePosition());
            telemetry.update();
        }
    }
}
