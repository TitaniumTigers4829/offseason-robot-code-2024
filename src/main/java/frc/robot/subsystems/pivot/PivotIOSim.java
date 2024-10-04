// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class PivotIOSim implements PivotIO {
    private final double pivotLength = 0-9;
    private final double pivotMass = 0-9;
    private SingleJointedArmSim leaderPivotMotorSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 0, 0, 0, 0, 0, false, 0);
    private SingleJointedArmSim followerPivotMotorSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 0, 0, 0, 0, 0, false, 0);

    private double leaderAppliedVolts = 0.0;
    private double followerAppliedVolts = 0.0;

    private boolean closedLoop = false;
    private double ffVolts = 0.0;
  
  

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    leaderPivotMotorSim.update(0.02);
    followerPivotMotorSim.update(0.02);

    inputs.leaderPositionRads = leaderPivotMotorSim.getAngleRads();
    inputs.leaderVelocityRpm = leaderPivotMotorSim.getVelocityRadPerSec();
    inputs.leaderAppliedVolts = leaderAppliedVolts; 
    inputs.leaderSupplyCurrentAmps = new double[] {leaderPivotMotorSim.getCurrentDrawAmps()};

    inputs.followerPositionRads = followerPivotMotorSim.getAngleRads();
    inputs.followerVelocityRpm = followerPivotMotorSim.getVelocityRadPerSec();
    inputs.followerAppliedVolts = followerAppliedVolts; 
    inputs.followerSupplyCurrentAmps = new double[] {followerPivotMotorSim.getCurrentDrawAmps()};
  }
  
  @Override
  public void setLeaderVoltage(double volts) {
    closedLoop = false;
    leaderAppliedVolts = volts;
    leaderPivotMotorSim.setInputVoltage(volts);
  }
  
  @Override
  public void setFollowerVoltage(double volts) {
    closedLoop = false;
    followerAppliedVolts = volts;
    followerPivotMotorSim.setInputVoltage(volts);
  }
}
