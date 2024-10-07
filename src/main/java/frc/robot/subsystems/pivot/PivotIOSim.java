// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.PivotConstants;


public class PivotIOSim implements PivotIO {
    private final double pivotLength = PivotConstants.PIVOT_LENGTH;
    private final double pivotMass = PivotConstants.PIVOT_MASS;
    private final double pivotGearing = PivotConstants.PIVOT_GEARING;

    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60(2), pivotGearing, pivotMass, pivotLength, 0, 0, true, 0);

    private double leaderAppliedVolts = 0.0;
    private double followerAppliedVolts = 0.0;
    
    private boolean closedLoop = false;
    private double ffVolts = 0.0;
  
  
  /**
   * Updates Inputs
   * @param inputs inputs for logging
   */
  @Override
  public void updateInputs(PivotIOInputs inputs) {
    pivotSim.update(0.02);

    inputs.leaderPosition = (pivotSim.getAngleRads()/Math.PI*2); // Rotations
    inputs.leaderVelocity = (pivotSim.getVelocityRadPerSec()/Math.PI*2); // Rotations/Sec
    inputs.leaderAppliedVolts = leaderAppliedVolts;; 
    inputs.leaderSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();

    inputs.followerPosition = (pivotSim.getAngleRads()/Math.PI*2); // Rotations
    inputs.followerVelocity = (pivotSim.getVelocityRadPerSec()/Math.PI*2); // Rotations/Sec
    inputs.followerAppliedVolts = followerAppliedVolts; 
    inputs.followerSupplyCurrentAmps =  pivotSim.getCurrentDrawAmps();
  }
  
  /** Sets Voltage */
  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    leaderAppliedVolts = volts;
    followerAppliedVolts = volts;
    pivotSim.setInputVoltage(volts);
  } 
}