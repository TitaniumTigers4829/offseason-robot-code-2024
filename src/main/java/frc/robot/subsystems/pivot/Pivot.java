// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  /** Creates a new Pivot. */
  public Pivot(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    // This method will be called once per scheduler run
  }

  public void setPivotAngle(double angle) {
    io.setPivotAngle(angle);
  }

  public void setPivotSpeed(double output) {
    io.setPivotSpeed(output);
  }

  public boolean isPivotWithinAcceptableError() {
    return io.isPivotWithinAcceptableError();
  }

  public double getAngle() {
    return io.getAngle();
  }

  public double getPivotTarget() {
    return io.getPivotTarget();
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }
}
