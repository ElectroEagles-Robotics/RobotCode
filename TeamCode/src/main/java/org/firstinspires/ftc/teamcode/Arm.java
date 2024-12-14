package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class Arm {
    private final DcMotorEx bottomMotor;
    private final DcMotor topMotor;
    private final CRServo intakeServo;
    private final Servo clawServo;

    private static final double ARM_POWER = 0.5;
    private static final double CLAW_POWER = 0.5;
    private static final double BRUSH_POWER = 1;
    private static final double ARM_STAY_POWER = 0.11;
    private static final int BOTTOM_MOTOR_TICKS_PER_REVOLUTION = 28 * 60;

    private static final double CLAW_OPEN_POSITION = 1;
    private static final double CLAW_CLOSE_POSITION = 0;

    public Arm(HardwareMap hardwareMap) {
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        topMotor = hardwareMap.get(DcMotor.class, "topMotor");
        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public void armUp() {
        bottomMotor.setPower(ARM_POWER);
    }

    public void armDown() {
        bottomMotor.setPower(-ARM_POWER);
    }

    public void armStay() {
        double rotations = (double) bottomMotor.getCurrentPosition() / BOTTOM_MOTOR_TICKS_PER_REVOLUTION;
        double motorPower;
        if (rotations > 0.5 && rotations < 0.8) {
            motorPower = ARM_STAY_POWER;
        } else {
            motorPower = 0;
        }

        bottomMotor.setPower(motorPower);
    }

    public void clawUp() {
        topMotor.setPower(CLAW_POWER);
    }

    public void clawDown() {
        topMotor.setPower(-CLAW_POWER);
    }

    public void clawStop() {
        topMotor.setPower(0);
    }

    public void brushIn() {
        intakeServo.setPower(BRUSH_POWER);
    }

    public void brushOut() {
        intakeServo.setPower(-BRUSH_POWER);
    }

    public void brushStop() {
        intakeServo.setPower(0);
    }



    private void clawOpen() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }

    private void clawClose() {
        clawServo.setPosition(CLAW_CLOSE_POSITION);
    }

    public void clawToggle() {
        double currentPosition = clawServo.getPosition();

        double distanceToOpenPosition = Math.abs(currentPosition - CLAW_OPEN_POSITION);
        double distanceToClosedPosition = Math.abs(currentPosition - CLAW_CLOSE_POSITION);

        if (distanceToOpenPosition > distanceToClosedPosition) {
            clawOpen();
        } else {
            clawClose();
        }
    }
}
