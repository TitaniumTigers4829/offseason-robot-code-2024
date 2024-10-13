package frc.robot.subsystems.swerve.gyroIO;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean isConnected = false;
    public Rotation2d yawDegrees = new Rotation2d();
    public double pitchDegrees = 0.0;
    public double rollDegrees = 0.0;
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double[] accel = new double[] {0, 0, 0};
    public double yawVelocity = 0.0;
    public boolean connected;
    public double[] odometryYawTimestamps;
    public Rotation2d yawPosition;
    public double yawVelocityRadPerSec;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void reset() {}

  public default void addOffset(Rotation2d offset) {}
}
