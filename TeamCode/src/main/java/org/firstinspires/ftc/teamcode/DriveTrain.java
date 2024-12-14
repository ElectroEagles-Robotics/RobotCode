package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class DriveTrain {
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;
    private final PIDController pidController = new PIDController(0, 0, 0);
    private final Gyro gyro;

    public DriveTrain(HardwareMap hardwareMap, Gyro gyro) {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        this.gyro = gyro;
    }

    public void joystickDrive(double verticalMovement, double horizontalMovement) {
        double currentAngle = gyro.getAngle();
        double correctionValue = pidController.update(currentAngle);

        double leftPower = Range.clip(verticalMovement + horizontalMovement + correctionValue, -1.0, 1.0) ;
        double rightPower = Range.clip(verticalMovement - horizontalMovement - correctionValue, -1.0, 1.0) ;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        if (horizontalMovement != 0) {
            pidController.setTargetValue(currentAngle);
        }
    }
}
