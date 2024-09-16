// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;


public interface PivotIO {
  @AutoLog
   class PivotIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;

    public double leftPositionRads = 0.0;
    public double leftVelocityRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTorqueCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightPositionRads = 0.0;
    public double rightVelocityRpm = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTorqueCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }
  /** Update inputs */
  default void updateInputs(PivotIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double leftVolts) {}

  /** Stop both flywheels */
  default void stop() {}

  /** Run left and right flywheels at velocity in rpm */
  default void runVelocity(double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {}

  /** Config PID values for both motors */
  default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for both motors */
  default void setFF(double kS, double kV, double kA) {}

  /** Run left flywheels at voltage */
  default void runCharacterizationLeft(double input) {}

  /** Run right flywheels at voltage */
  default void runCharacterizationRight(double input) {}
}

  

