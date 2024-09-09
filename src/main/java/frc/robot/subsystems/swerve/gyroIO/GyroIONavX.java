package frc.robot.subsystems.swerve.gyroIO;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.swerve.gyroIO.GyroIO.GyroIOInputs;

public class GyroIONavX implements GyroIO {
    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 250);

    public GyroIONavX() {
        reset();
    }
    
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = gyro.getAngle();
        inputs.yawVelocity = gyro.getRate();
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    public void setOffset(double offset) {
        gyro.setAngleAdjustment(offset);
    }
}