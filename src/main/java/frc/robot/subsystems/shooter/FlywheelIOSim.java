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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO { //FlywheelIOSim makes Advantage kit log simulated movements using physics 
  private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getKrakenX60(2), FlywheelConstants.GEAR_RATIO, 0.004); //initiates virtual
  private DIOSim noteSensorSim = new DIOSim(new DigitalInput(FlywheelConstants.NOTE_SENSOR_ID));
  private PIDController pid = new PIDController(FlywheelConstants.FLYWHEEL_P, FlywheelConstants.FLYWHEEL_I, FlywheelConstants.FLYWHEEL_D);
  private SimpleMotorFeedforward feedFoward = new SimpleMotorFeedforward(FlywheelConstants.FLYWHEEL_S, FlywheelConstants.FLYWHEEL_V, FlywheelConstants.FLYWHEEL_A);
  
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    flywheelSim.update(0.02);

    inputs.positionRotations = 0.0;
    inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.isNoteDetected = hasNote();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    flywheelSim.setInputVoltage(volts);
  }
/**
 * @param velocityRPM User inputs the desired velocity in RPM, gets converted in method for PID to set value in RPS 
 */
  @Override
  public void setVelocity(double velocityRPM) { //ffvolts is feedorward
    flywheelSim.setState(pid.calculate(velocityRPM / 60.0));
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  public boolean hasNote() {
    return !noteSensorSim.getValue();
  }
}