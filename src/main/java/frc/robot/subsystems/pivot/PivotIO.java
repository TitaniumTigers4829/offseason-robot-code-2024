package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;
    public double leaderPosition = 0.0;
    public double leaderVelocity = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    public double followerPosition = 0.0;
    public double followerVelocity = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  /**
   * Gets the angle of the pivot
   *
   * @return angle of pivot in rotations
   */
  public default double getAngle() {
    return 0.0;
  }

  public default boolean isPivotWithinAcceptableError() {
    return false;
  }

  /**
   * Sets the output of the pivot
   *
   * @param output output value from -1.0 to 1.0
   */
  public default void setPivotSpeed(double output) {}

  /**
   * Sets the voltage of the pivot motors
   *
   * @param volts the voltage
   */
  public default void setVoltage(double volts) {}

  /**
   * sets the pivot angle(rotations) shooter
   *
   * @param angle the desired angle in rotations
   */
  public default void setPivotAngle(double angle) {}

  /**
   * Gets the target angle of the pivot in degrees
   *
   * @return the target angle
   */
  public default double getPivotTarget() {
    return 0.0;
  }
}
