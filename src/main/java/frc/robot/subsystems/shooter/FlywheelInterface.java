package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelInterface {
  @AutoLog
  public static class FlywheelInputs { // variables that we want to log
    public double positionRotations = 0.0; // positions in radians | convert to rpms
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public boolean isNoteDetected = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Run closed loop at the specified velocity.
   *
   * @param velocityRPM Recieve desired input in rounds per minute, and the method will convert to
   *     RPS to match requirements for VelocityVoltage
   */
  public default void setVelocity(double velocityRPM) {}

  /** Stop in open loop. */
  public default void stop() {}

  default void setRollerSpeed(double speed) {}
}
