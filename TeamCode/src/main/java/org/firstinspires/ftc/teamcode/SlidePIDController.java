package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlidePIDController {
    private DcMotor slideMotor;
    private final double kP = 0.6;
    private final double kI = 0.02;
    private final double kD = 0;
    private final double kStatic = 0;
    private double lastError;
    private double integralSum;
    private double target;
    private ElapsedTime timer;

    public SlidePIDController(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        lastError = 0.0;
        integralSum = 0.0;
        target = 0.0;
        timer = new ElapsedTime();
    }

    public void setTarget(double targetLevel) {
        target = targetLevel;
        timer.reset();
    }

    public double getSlidePosition() {
        return slideMotor.getCurrentPosition() / 537.7; // normalized by ticks per rev
    }

    public double update() {
        final double state = getSlidePosition();

        double error = target - state;
        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        double out = (kP * error) + (kI * integralSum) + (kD * derivative) + kStatic;

        lastError = error;

        timer.reset();

        slideMotor.setPower(out);

        return out;
    }
}
