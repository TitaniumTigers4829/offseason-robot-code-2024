// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.HardwareConstants;

public class PivotIOSim implements PivotIO {
  private final double pivotGearing = PivotConstants.PIVOT_GEARING;
  private final double pivotMass = PivotConstants.PIVOT_MASS;
  private final double pivotLength = PivotConstants.PIVOT_LENGTH;
  private final double armkS = PivotConstants.ARM_KS;
  private final double armkG = PivotConstants.ARM_KG;
  private final double armkV = PivotConstants.ARM_KV;

  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2), pivotGearing, pivotMass, pivotLength, 0, 0, true, 0);

  private final Constraints pivotConstraints = new Constraints(0, 0);
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
    pivotSim.update(HardwareConstants.RIO_UPDATE_SECONDS);

    inputs.leaderPosition = Units.radiansToRotations(pivotSim.getAngleRads());
    inputs.leaderVelocity = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());
    inputs.leaderAppliedVolts = leaderAppliedVolts;
    inputs.leaderSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();

    inputs.followerPosition = Units.radiansToRotations(pivotSim.getAngleRads());
    inputs.followerVelocity = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());
    inputs.followerAppliedVolts = followerAppliedVolts;
    inputs.followerSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();
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
    setVoltage(pivotController.calculate(angleRots, currentPivotAngleRots) + armFF);
  }
}
