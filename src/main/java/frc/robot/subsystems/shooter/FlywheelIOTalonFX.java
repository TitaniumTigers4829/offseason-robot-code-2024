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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = FlywheelConstants.GEAR_RATIO;

  private final TalonFX leaderFlywheelMotor = new TalonFX(FlywheelConstants.LEADER_FLYWHEEL_MOTOR_ID); //Leader=left motor
  private final TalonFX followerFlywheelMotor = new TalonFX(FlywheelConstants.FOLLOWER_FLYWHEEL_MOTOR_ID); //follower = right motor

  private final StatusSignal<Double> leaderPosition = leaderFlywheelMotor.getPosition();
  private final StatusSignal<Double> leaderVelocity = leaderFlywheelMotor.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leaderFlywheelMotor.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leaderFlywheelMotor.getSupplyCurrent();
  private final StatusSignal<Double> followerCurrent = followerFlywheelMotor.getSupplyCurrent();

  public FlywheelIOTalonFX() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderFlywheelMotor.getConfigurator().apply(flywheelConfig);
    followerFlywheelMotor.getConfigurator().apply(flywheelConfig);
    followerFlywheelMotor.setControl(new Follower(leaderFlywheelMotor.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leaderFlywheelMotor.optimizeBusUtilization();
    followerFlywheelMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    leaderFlywheelMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leaderFlywheelMotor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() {
    leaderFlywheelMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leaderFlywheelMotor.getConfigurator().apply(config);
  }
}