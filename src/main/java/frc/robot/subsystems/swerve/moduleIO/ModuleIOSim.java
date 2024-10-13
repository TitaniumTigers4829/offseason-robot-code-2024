package frc.robot.subsystems.swerve.moduleIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.extras.simulation.physicsSim.SwerveModuleSimulation;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

import java.util.Arrays;

/** Wrapper class around {@link SwerveModuleSimulation} that implements ModuleIO */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;

  private final PIDController drivePID = new PIDController(5, 0, 0);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(5, 0, 0);
  private final Constraints turnConstraints = new Constraints(5, 0);
  private final ProfiledPIDController turnPID = new ProfiledPIDController(5, 0, 0, turnConstraints);
  private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(5, 0, 0);

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // inputs.drivePositionRad = moduleSimulation.getDriveEncoderFinalPositionRad();
    inputs.driveVelocity =
        moduleSimulation.getDriveWheelFinalSpeedRadPerSec(); // TODO: Convert from radians to meters
    inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps());

    inputs.turnAbsolutePosition = moduleSimulation.getTurnAbsolutePosition();
    inputs.turnPosition =
        Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderPositionRad());
    inputs.turnVelocityRadPerSec = moduleSimulation.getSteerRelativeEncoderSpeedRadPerSec();
    inputs.turnAppliedVolts = moduleSimulation.getSteerMotorAppliedVolts();
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorSupplyCurrentAmps());

    inputs.odometryTimestamps = OdometryTimestampsSim.getTimeStamps();
    inputs.odometryDrivePositionsRad = moduleSimulation.getCachedDriveWheelFinalPositionsRad();
    inputs.odometryTurnPositions =
        Arrays.stream(moduleSimulation.getCachedSteerRelativeEncoderPositions())
            .mapToObj(Rotation2d::fromRadians)
            .toArray(Rotation2d[]::new);

            inputs.odometryDriveWheelRevolutions = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositionsRad())
            .map(Units::radiansToRotations)
            .toArray();

  }

  @Override
  public void setDriveVoltage(double volts) {
    moduleSimulation.requestDriveVoltageOut(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    moduleSimulation.requestTurnVoltageOut(volts);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    double turnRotations = getTurnRotations();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(turnRotations));

    if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01) {
      moduleSimulation.requestDriveVoltageOut(0);
      moduleSimulation.requestTurnVoltageOut(0);
      return;
    }

    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        optimizedDesiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    moduleSimulation.requestDriveVoltageOut(
        drivePID.calculate(
                Units.radiansToRotations(moduleSimulation.getDriveWheelFinalSpeedRadPerSec()),
                desiredDriveRPS)
            + driveFF.calculate(desiredDriveRPS));
    moduleSimulation.requestTurnVoltageOut(
        turnPID.calculate(
                moduleSimulation.getTurnAbsolutePosition().getRotations(),
                desiredState.angle.getRotations())
            + turnFF.calculate(turnPID.getSetpoint().velocity));
  }

  public double getTurnRotations() {
    return moduleSimulation.getTurnAbsolutePosition().getRotations();
  }
}
