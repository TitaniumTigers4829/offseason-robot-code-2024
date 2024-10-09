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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = FlywheelConstants.GEAR_RATIO; //Gear ratio of two motors on flywheel

  private final TalonFX leaderFlywheelMotor = new TalonFX(FlywheelConstants.LEADER_FLYWHEEL_MOTOR_ID); //Leader=left motor
  private final TalonFX followerFlywheelMotor = new TalonFX(FlywheelConstants.FOLLOWER_FLYWHEEL_MOTOR_ID); //follower = right motor
// gives values for each thing that is set. 
  private final StatusSignal<Double> leaderPosition = leaderFlywheelMotor.getPosition(); 
  private final StatusSignal<Double> leaderVelocity = leaderFlywheelMotor.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leaderFlywheelMotor.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leaderFlywheelMotor.getSupplyCurrent();
  private final StatusSignal<Double> followerCurrent = followerFlywheelMotor.getSupplyCurrent(); //All of these are from TalonFX Phoenix 6 that assign values to advantagekit log variables

  public FlywheelIOTalonFX() {  //Object to set different flywheel configs 
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.FLYWHEEL_SUPPLY_LIMIT; //Talonfx configuration software limits
    flywheelConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.FLYWHEEL_STATOR_LIMIT;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = FlywheelConstants.FLYWHEEL_SUPPLY_ENABLE;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = FlywheelConstants.FLYWHEEL_STATOR_ENABLE;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; //This is used to ensure maximum power efficiency, to ensure flywheels keep spinning after power off.
 
    leaderFlywheelMotor.getConfigurator().apply(flywheelConfig); //Apply the flywheel config defined above
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //invert one so that they can go in the same direction
    followerFlywheelMotor.getConfigurator().apply(flywheelConfig);
    followerFlywheelMotor.setControl(new Follower(leaderFlywheelMotor.getDeviceID(), false)); //phoenix 6 method to make the follower motor to copy, so code only needed to write for leader. NOT NECESSARY, JUST CONVENTION STOLEN EASTBOT

    BaseStatusSignal.setUpdateFrequencyForAll( //sets the frequency where the values are updated
        250.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leaderFlywheelMotor.optimizeBusUtilization(); //updates frequency based on how often the bus communicates with the motor
    followerFlywheelMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {//gets current values for motors and puts them into a log
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRotaions = leaderPosition.getValueAsDouble() / GEAR_RATIO; //converted to radians to gear ratio math
    inputs.velocityRPM = (leaderVelocity.getValueAsDouble() * 60.0) / GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()}; //puts current values into an array as doubles
  }

  @Override
  public void setVoltage(double volts) { //some method to set voltage for motor
    leaderFlywheelMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRPM, double ffVolts) {//updating voltage to alter velocity of the motor
    leaderFlywheelMotor.setControl(
        new VelocityVoltage(
            velocityRPM / 60.0,
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() { //stops the motor
    leaderFlywheelMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) { //sets the pid values
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    leaderFlywheelMotor.getConfigurator().apply(config);//applies configurations to the motor
  }
}