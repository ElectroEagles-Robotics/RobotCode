package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final ElapsedTime runtime = new ElapsedTime();
    private final double kP;
    private final double kI;
    private final double kD;
    private final double MAX_INTEGRAL_SUM;

    private double targetValue = 0;
    private double integralSum = 0;
    private double lastError = 0;

    public PIDController(double kP, double kI, double kD, double MAX_INTEGRAL_SUM) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.MAX_INTEGRAL_SUM = MAX_INTEGRAL_SUM;
    }

    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, Double.POSITIVE_INFINITY);
    }


    public double update(double currentValue) {
        double error = targetValue - currentValue;
        integralSum += error * runtime.seconds();

        if (integralSum > MAX_INTEGRAL_SUM) {
            integralSum = MAX_INTEGRAL_SUM;
        } else if (integralSum < -MAX_INTEGRAL_SUM) {
            integralSum = -MAX_INTEGRAL_SUM;
        }

        double derivative = (error - lastError) / runtime.seconds();

        double correction = kP * error + kI * integralSum + kD * derivative;

        lastError = error;
        runtime.reset();

        return correction;
    }

    public void setTargetValue(double targetValue) {
        this.targetValue = targetValue;
    }

}
