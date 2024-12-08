package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Gyro {
    private final BHI260IMU imu;

    public Gyro(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BHI260IMU.class, "imu");
    }

    public double getAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
