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

// Modified by 5516 "IRON MAPLE", original source:
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim

package frc.robot.subsystems.swerve.moduleIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.physicsSim.SwerveModuleSimulation;

import java.util.Arrays;

/** Wrapper class around {@link SwerveModuleSimulation} that implements ModuleIO */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;

  private final PIDController drivePID = new PIDController(0,0,0);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0,0,0);
  private final Constraints turnConstraints = new Constraints(0,0);
  private final ProfiledPIDController turnPID = new ProfiledPIDController(0,0,0, turnConstraints);
  private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0,0,0);

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = moduleSimulation.getDriveEncoderFinalPositionRad();
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeedRadPerSec();
    inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
    inputs.driveCurrentAmps =
        new double[] {Math.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps())};

    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnPosition =
        Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderPositionRad());
    inputs.turnVelocityRadPerSec = moduleSimulation.getSteerRelativeEncoderSpeedRadPerSec();
    inputs.turnAppliedVolts = moduleSimulation.getSteerMotorAppliedVolts();
    inputs.turnCurrentAmps =
        new double[] {Math.abs(moduleSimulation.getSteerMotorSupplyCurrentAmps())};

    inputs.odometryTimestamps = OdometryTimestampsSim.getTimeStamps();
    inputs.odometryDrivePositionsRad = moduleSimulation.getCachedDriveWheelFinalPositionsRad();
    inputs.odometryTurnPositions =
        Arrays.stream(moduleSimulation.getCachedSteerRelativeEncoderPositions())
            .mapToObj(Rotation2d::fromRadians)
            .toArray(Rotation2d[]::new);
  }

  @Override
  public void setDriveVoltage(double volts) {
    moduleSimulation.requestDriveVoltageOut(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    moduleSimulation.requestSteerVoltageOut(volts);
  }
  
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(desiredState, desiredState.angle);

    if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01) {
      moduleSimulation.requestDriveVoltageOut(0);;
      moduleSimulation.requestSteerVoltageOut(0);
      return;
    }

    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        optimizedDesiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    moduleSimulation.requestDriveVoltageOut(drivePID.calculate(moduleSimulation.getDriveWheelFinalSpeedRadPerSec(), desiredDriveRPS) + driveFF.calculate(desiredDriveRPS));
    moduleSimulation.requestSteerVoltageOut(turnPID.calculate(moduleSimulation.getSteerAbsoluteFacing().getRotations(), desiredState.angle.getRotations()) + turnFF.calculate(desiredDriveRPS));
    
  }
}