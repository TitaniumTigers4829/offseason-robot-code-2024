// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  // private final SimpleMotorFeedforward ffModel;
  
  public Flywheel(FlywheelIO io) {
    this.io = io;
    // switch (HardwareConstants.currentMode) {
    //   case REAL:
    //   case REPLAY:
    //     ffModel = new SimpleMotorFeedforward(0.1, 0.05);
    //     break;
    //   case SIM:
    //     ffModel = new SimpleMotorFeedforward(0.0, 0.03);
    //     break;
    //   default:
    //     ffModel = new SimpleMotorFeedforward(0.0, 0.0);
    //     break;
    // }
  }

  public void setFlywheelVoltage(double desiredVoltage) {
    io.setVoltage(desiredVoltage);
  }

  public void setFlywheelVelocity(double desiredRPM, double ffVolts) {
    io.setVelocity(desiredRPM, ffVolts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("FlywheelSubsystem", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
