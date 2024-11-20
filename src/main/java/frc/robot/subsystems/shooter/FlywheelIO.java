package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  /** Creates new variables necessary for logging */
  public static class FlywheelIOInputs { // Variables that we want to log
    public double positionRotations = 0.0; // Positions in radians | convert to rpms
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public boolean isNoteDetected = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Run closed loop at the specified velocity.
   *
   * @param velocityRPM Recieve desired input in rotations per minute
   */
  public default void setVelocity(double velocityRPM) {}

  /** Stop in open loop. */
  public default void stop() {}
  /**
   * Sets speed of the roller
   * @param speed Range is -1 to 1
   */
  default void setRollerSpeed(double speed) {}
}
