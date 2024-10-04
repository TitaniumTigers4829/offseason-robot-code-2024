// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;


public interface PivotIO {
  @AutoLog
   class PivotIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;
    public double leaderPositionRads = 0.0;
    public double leaderVelocityRpm = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double[] leaderSupplyCurrentAmps = null;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    public double followerPositionRads = 0.0;
    public double followerVelocityRpm = 0.0;
    public double followerAppliedVolts = 0.0;
    public double[] followerSupplyCurrentAmps = null;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }
  /** Update inputs */
  public default void updateInputs(PivotIOInputs inputs) {}

  public default double getAngle() {
    return 0.0;
  }

  public default boolean isPivotWithinAcceptableError() {
    return false;
  }

  public default void setPivotFromSpeakerDistance(double speakerDistance) {}

  public default void setPivotSpeed(double output) {}

  public default void setLeaderVoltage(double volts) {}

  public default void setFollowerVoltage(double volts) {}

  public default void setPivotFromPassDistance(double passDistance) {}
  
  public default void setPivotAngle(double angle) {}

  public default double getPivotTarget() {
    return 0.0;
  }


}

  

