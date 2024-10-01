package frc.robot.subsystems.swerve.gyroIO;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.swerve.gyroIO.GyroIO.GyroIOInputs;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;

public class GyroIONavX implements GyroIO {
    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 250);

    public GyroIONavX() {
        // reset();
        // gyro.enableLogging(true);
        OdometryThread.registerInput(getAngle());
    }
    
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = getGyroRotation2d();
        inputs.yawVelocity = getRate();
    }

    // @Override
    public void zeroHeading() {
        gyro.reset();
    }

    public void setOffset(double offset) {
        gyro.setAngleAdjustment(offset);
    }

    public Supplier<Double> getAngle() {
       return ()-> -gyro.getAngle();
    }

    public double getYaw() {
       return -gyro.getAngle(); 
    }
    public Rotation2d getGyroRotation2d() {
        return gyro.getRotation2d();
    }

    public double getRate() {
        return -gyro.getRate();
    }

    public double getAllianceAngleOffset() {
        return 180;
    }

    public Rotation2d getGyroFieldRelativeRotation2d() {
        return getGyroRotation2d().plus(Rotation2d.fromDegrees(getAllianceAngleOffset()));
    }

}