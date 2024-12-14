package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class DriveTrain {
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;
    private final PIDController pidController = new PIDController(0.02, 0, 0);
    private final Gyro gyro;

    public DriveTrain(HardwareMap hardwareMap, Gyro gyro) {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        pidController.setTargetValue(gyro.getAngle());
        this.gyro = gyro;
    }

    public void joystickDrive(double verticalMovement, double horizontalMovement, Telemetry telemetry) {
        double currentAngle = gyro.getAngle();
        double correctionValue;

        if (horizontalMovement != 0) {
            pidController.setTargetValue(currentAngle);
            correctionValue = 0;
        } else {
            correctionValue = pidController.update(currentAngle);
        }

        telemetry.addData("Correction", correctionValue);
        telemetry.addData("Angle", currentAngle);
        telemetry.addData("Target Angle", pidController.getTargetValue());

        double leftPower = Range.clip(verticalMovement + horizontalMovement - correctionValue, -1.0, 1.0) ;
        double rightPower = Range.clip(verticalMovement - horizontalMovement + correctionValue, -1.0, 1.0) ;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}
