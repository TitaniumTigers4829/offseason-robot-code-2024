// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  public Flywheel() {}

  public void setFlywheelVoltage(double desiredVoltage) {}

  public void setFlywheelVelocity(double desiredRPM) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}