package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlidePIDController {
    private final DcMotor slideMotor;
    private final double kP = 0.55;
    private final double kI = 0;
    private final double kD = 0;
    private final double kStatic = 0.1;
    private double target = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private double forcedPower = 0;
    private boolean beingForced = false;
    private final ElapsedTime timer;

    public SlidePIDController(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        timer = new ElapsedTime();
    }

    public void resetEncoder() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(double targetLevel) {
        target = targetLevel;
        lastError = 0;
        integralSum = 0;
        beingForced = false;
        timer.reset();
    }

    public void setPower(double power) {
        forcedPower = power;
        beingForced = true;
    }

    public double getSlidePosition() {
        return slideMotor.getCurrentPosition() / 537.7; // normalized by ticks per rev
    }

    public double update() {
        if (beingForced) {
            slideMotor.setPower(forcedPower);
            return forcedPower;
        }

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
