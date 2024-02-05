// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.extras.SingleLinearInterpolator;

public class ShooterSubsystem extends SubsystemBase {
  private final DigitalInput noteSensor;
  private final TalonFX leaderFlywheel;
  private final TalonFX followerFlywheel;
  private final TalonFX rollerMotor;

  private double leftMotorTargetRPM;

  StatusSignal<Double> shooterVelocity;

  private SingleLinearInterpolator speakerSpeedValues;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() { 
    leaderFlywheel = new TalonFX(ShooterConstants.LEADER_FLYWHEEL_ID);
    followerFlywheel = new TalonFX(ShooterConstants.FOLLOWER_FLYWHEEL_ID);
    rollerMotor = new TalonFX(ShooterConstants.ROLLER_MOTOR_ID);
    noteSensor = new DigitalInput(ShooterConstants.SHOOTER_NOTE_SENSOR_ID);

    speakerSpeedValues = new SingleLinearInterpolator(ShooterConstants.SPEAKER_SHOOT_RPMS);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    shooterConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    shooterConfig.Slot0.kD = ShooterConstants.SHOOT_D;

    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    shooterConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    leaderFlywheel.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    followerFlywheel.getConfigurator().apply(shooterConfig);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerMotor.getConfigurator().apply(rollerConfig);

    shooterVelocity = leaderFlywheel.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(250, shooterVelocity);
    leaderFlywheel.optimizeBusUtilization(HardwareConstants.TIMEOUT_S);
  }

  public double getLeftShooterRPM() {
    shooterVelocity.refresh();
    return shooterVelocity.getValueAsDouble() * 60;
  }

  public void setShooterSpeed(double speed) {
    leaderFlywheel.set(speed);
    Follower follower = new Follower(leaderFlywheel.getDeviceID(), true);
    followerFlywheel.setControl(follower);
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public boolean isShooterWithinAcceptableError() {
    return Math.abs(leftMotorTargetRPM - getLeftShooterRPM()) < 20;
  }

  public void setRPM(double leftRPMMotor) {
    leftMotorTargetRPM = leftRPMMotor;
    VelocityVoltage leftSpeed = new VelocityVoltage(leftRPMMotor / 60.0);
    leaderFlywheel.setControl(leftSpeed);
    Follower follower = new Follower(leaderFlywheel.getDeviceID(), true);
    followerFlywheel.setControl(follower);
  }

  public void setLeftMotorToNeutral() {
    leaderFlywheel.set(0);
  }

  /**
   * Gets the sensor in the shooter pivot
   * @return True if nothing is detected
   */
  public boolean getSensor() {
    return noteSensor.get();
  }

  public void setShooterRPMFromDistance(double distance) {
    double rpm = speakerSpeedValues.getLookupValue(distance);
    setRPM(rpm);
  }
  
  @Override
  public void periodic() {
  }

}