// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HardwareConstants;

public class PivotIOSim implements PivotIO {
  private final double pivotGearing = PivotConstants.PIVOT_GEAR_RATIO;
  private final double pivotMass = PivotConstants.PIVOT_MASS;
  private final double pivotLength = PivotConstants.PIVOT_LENGTH;
  private final double armkS = 0.0;
  private final double armkG = PivotConstants.PIVOT_G;
  private final double armkV = 0.0;

  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          pivotGearing,
          0.01,
          pivotLength,
          Rotations.of(0).in(Radians),
          Rotations.of(1).in(Radians),
          true,
          Rotations.of(0).in(Radians));

  private final Constraints pivotConstraints = new Constraints(0.8, 0.8);
  private final ArmFeedforward armFeedforward = new ArmFeedforward(armkS, armkG, armkV);
  private final ProfiledPIDController pivotController =
      new ProfiledPIDController(0, 0, 0, pivotConstraints);
  private double leaderAppliedVolts = 0.0;
  private double followerAppliedVolts = 0.0;

  /**
   * Updates inputs for logging w/ advantage kit
   *
   * @param inputs inputs for logging
   */
  @Override
  public void updateInputs(PivotIOInputs inputs) {
    pivotSim.update(HardwareConstants.TIMEOUT_S);

    inputs.leaderPosition = Units.radiansToRotations(pivotSim.getAngleRads());
    inputs.leaderVelocity = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());
    inputs.leaderAppliedVolts = leaderAppliedVolts;
    inputs.leaderSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();

    inputs.followerPosition = Units.radiansToRotations(pivotSim.getAngleRads());
    inputs.followerVelocity = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());
    inputs.followerAppliedVolts = followerAppliedVolts;
    inputs.followerSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();
    SmartDashboard.putNumber("Pivot Angle Rads", pivotSim.getAngleRads());
    SmartDashboard.putNumber("Pivot Speed Rads Per Sec", pivotSim.getVelocityRadPerSec());
  }

  /**
   * Sets Voltage of the shooter
   *
   * @param volts desired voltage
   */
  @Override
  public void setVoltage(double volts) {
    leaderAppliedVolts = volts;
    followerAppliedVolts = volts;
    pivotSim.setInputVoltage(volts);
  }

  /**
   * sets the Pivot angle
   *
   * @param angleRots desired angle of shooter in rotations
   */
  @Override
  public void setPivotAngle(double angleRots) {
    double currentPivotAngleRots = Units.radiansToRotations(pivotSim.getAngleRads());
    double armFF = armFeedforward.calculate(angleRots, pivotController.getSetpoint().velocity);
    setVoltage(pivotController.calculate(currentPivotAngleRots, angleRots) + armFF);
  }

  @Override
  public void setPivotSpeed(double output) {
    pivotSim.setInput(output);
  }
}
