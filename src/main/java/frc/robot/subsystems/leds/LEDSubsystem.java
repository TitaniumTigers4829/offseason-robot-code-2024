// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;


public class LEDSubsystem extends SubsystemBase {
  private Spark ledSpark;

  private LEDProcess process;

  /** Creates a new LEDSubsystemImpl for use with LED strips made by Spark lighting (Loopy's LEDs).
   * @param port The Spark port for this LEDSubsystem.
  */
  public LEDSubsystem(int port) {
    ledSpark = new Spark(port);
    setProcess(LEDProcess.OFF);
  }

  /** Creates a new LEDSubsystemImpl with the port in LEDConstants. */
  public LEDSubsystem() {
    ledSpark = new Spark(LEDConstants.LEDPort);
    setProcess(LEDProcess.OFF);
  }
  
  @Override
  public void periodic() {
    
  }
  
  public void setProcess(LEDProcess process) {
    this.process = process;
  }

  public void off() {
    process = LEDProcess.OFF;
  }

}