// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.interpolators.SingleLinearInterpolator;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new Pivot. */

  private final CANcoder pivotEncoder;
  private final TalonFX leaderPivotMotor;
  private final TalonFX followerPivotMotor;

  private final MotionMagicVoltage mmPositionRequest;
  private final SingleLinearInterpolator speakerAngleLookupValues;
  private final SingleLinearInterpolator passAngleLookupValues;
  private final StatusSignal<Double> pivotPos;
  private double pivotTriangleAngle;

  public PivotSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
