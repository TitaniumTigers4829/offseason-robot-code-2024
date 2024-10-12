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
  }

  public void setFlywheelVoltage(double desiredVoltage) {
    io.setVoltage(desiredVoltage); ///io calls the functions
  }

  public void setFlywheelVelocity(double desiredRPM) {
    io.setVelocity(desiredRPM);
  }

  public boolean hasNote(){
    return inputs.isNoteDetected;
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
    io.updateInputs(inputs);
  }
}
