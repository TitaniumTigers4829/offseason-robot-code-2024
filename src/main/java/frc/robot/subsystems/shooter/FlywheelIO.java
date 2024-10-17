// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs { // variables that we want to log
    public double positionRotations = 0.0; // positions in radians | convert to rpms
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
   * @param velocityRPM Recieve desired input in rounds per minute, and the method will convert to
   *     RPS to match requirements for VelocityVoltage
   */
  public default void setVelocity(double velocityRPM) {}

  /** Stop in open loop. */
  public default void stop() {}

  default void setRollerSpeed(double speed) {}
}
