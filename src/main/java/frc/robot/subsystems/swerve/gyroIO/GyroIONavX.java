package frc.robot.subsystems.swerve.gyroIO;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.extras.util.AllianceFlipper;
// import frc.robot.extras.util.FieldMirroringUtils;
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
        inputs.isConnected = gyro.isConnected();
        inputs.yawDegrees = getGyroRotation2d();
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

    public Rotation2d getGyroFieldRelativeRotation2d() {
        if (AllianceFlipper.isBlue()) {
            return getGyroRotation2d();
        }
        return AllianceFlipper.flipRotation(getGyroRotation2d());
    }

}