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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.Constants.HardwareConstants;

public class FlywheelIOTalonFX implements FlywheelIO { // FlywheelIOTalonFX makes Advantagekit log physical hardware movement
  private final TalonFX leaderFlywheelMotor = new TalonFX(ShooterConstants.LEADER_FLYWHEEL_MOTOR_ID); // Leader=left motor
  private final TalonFX followerFlywheelMotor = new TalonFX(ShooterConstants.FOLLOWER_FLYWHEEL_MOTOR_ID); // follower = right motor
  private final DigitalInput noteSensor = new DigitalInput(ShooterConstants.NOTE_SENSOR_ID); // Note sensor

  private final VelocityVoltage velocityRequest;
  private final VoltageOut voltageRequest;
  // gives values for each thing that is set. 
  private final StatusSignal<Double> leaderPosition = leaderFlywheelMotor.getPosition(); 
  private final StatusSignal<Double> leaderVelocity = leaderFlywheelMotor.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leaderFlywheelMotor.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leaderFlywheelMotor.getSupplyCurrent();
  private final StatusSignal<Double> followerCurrent = followerFlywheelMotor.getSupplyCurrent(); // All of these are from TalonFX Phoenix 6 that assign values to advantagekit log variables

  public FlywheelIOTalonFX() {  // Object to set different flywheel configs 
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FLYWHEEL_SUPPLY_LIMIT; // Talonfx configuration software limits found in CONSTANTS file
    flywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_STATOR_LIMIT;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.FLYWHEEL_SUPPLY_ENABLE;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterConstants.FLYWHEEL_STATOR_ENABLE;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // This is used to ensure maximum power efficiency, to ensure flywheels keep spinning after power off.

    flywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_P; //everything tuned in Flywheel Constants file
    flywheelConfig.Slot0.kI = ShooterConstants.FLYWHEEL_I;
    flywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_D;
    flywheelConfig.Slot0.kS = ShooterConstants.FLYWHEEL_S;
    flywheelConfig.Slot0.kV = ShooterConstants.FLYWHEEL_V;
    flywheelConfig.Slot0.kA = ShooterConstants.FLYWHEEL_A;
 
    leaderFlywheelMotor.getConfigurator().apply(flywheelConfig); // Apply the flywheel config defined above
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // invert one so that they can go in the same direction
    followerFlywheelMotor.getConfigurator().apply(flywheelConfig);
    followerFlywheelMotor.setControl(new Follower(leaderFlywheelMotor.getDeviceID(), false)); // phoenix 6 method to make the follower motor to copy, so code only needed to write for leader. NOT NECESSARY, JUST CONVENTION STOLEN EASTBOT
    
    // sets the frequency where the values are updated
    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.SIGNAL_FREQUENCY, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leaderFlywheelMotor.optimizeBusUtilization(); // updates frequency based on how often the bus communicates with the motor
    followerFlywheelMotor.optimizeBusUtilization();

    velocityRequest = new VelocityVoltage(0.0);
    voltageRequest = new VoltageOut(0.0);
  }
/**
 * @param inputs has all the inputs, updates and logs them
 */
  @Override
  public void updateInputs(FlywheelIOInputs inputs) {// gets current values for motors and puts them into a log
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRotations = leaderPosition.getValueAsDouble() / ShooterConstants.GEAR_RATIO; // converted to radians to gear ratio math
    inputs.velocityRPM = (leaderVelocity.getValueAsDouble() * 60.0) / ShooterConstants.GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()}; // puts current values into an array as doubles
    inputs.isNoteDetected = hasNote();
  }

  @Override
  public void setVoltage(double volts) { // some method to set voltage for motor
    leaderFlywheelMotor.setControl(voltageRequest.withOutput(volts));
  }
/**
 * @param velocityRPM Recieve desired input in rounds per minute, and the method will convert to RPS to match requirements for VelocityVoltage * 
 */
  @Override
  public void setVelocity(double velocityRPM) {
    leaderFlywheelMotor.setControl(velocityRequest.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void stop() { // stops the motor
    leaderFlywheelMotor.stopMotor();
  }
  /**
   * @return Whether the sensor can detect a note in there
   */
  public boolean hasNote() {
    return !noteSensor.get();
  }
}