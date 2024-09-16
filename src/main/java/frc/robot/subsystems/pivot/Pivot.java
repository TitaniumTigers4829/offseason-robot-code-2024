// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HardwareConstants;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  
  private final SysIdRoutine sysId;
  /** Creates a new Pivot. */
  public Pivot(PivotIO io) {
    this.io = io;

  switch (HardwareConstants.currentMode) {
    case REAL:
    case REPLAY:
      //Tuning for replay values here
      break;
    case SIM:
      //Tuning for SIM here
      break;
    default:
      //idk default tuning here ig
      break;
  }
  sysId =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> io.runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
