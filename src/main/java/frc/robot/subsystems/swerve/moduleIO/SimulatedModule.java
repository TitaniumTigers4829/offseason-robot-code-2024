package frc.robot.subsystems.swerve.moduleIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

/** Wrapper class around {@link SwerveModuleSimulation} */
public class SimulatedModule implements ModuleInterface {
  private final SwerveModuleSimulation moduleSimulation;

  private final PIDController drivePID = new PIDController(.05, 0, 0);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 0.13);

  private final Constraints turnConstraints =
      new Constraints(
          RadiansPerSecond.of(2 * Math.PI).in(RotationsPerSecond),
          RadiansPerSecondPerSecond.of(4 * Math.PI).in(RotationsPerSecondPerSecond));
  private final ProfiledPIDController turnPID =
      new ProfiledPIDController(Radians.of(30).in(Rotations), 0, 0, turnConstraints);
  private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0.77, 0.75, 0);

  public SimulatedModule(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    inputs.drivePosition =
        Radians.of(moduleSimulation.getDriveEncoderFinalPositionRad()).in(Rotations);
    inputs.driveVelocity =
        RadiansPerSecond.of(moduleSimulation.getDriveWheelFinalSpeedRadPerSec())
            .in(RotationsPerSecond);
    inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
    inputs.driveCurrentAmps = moduleSimulation.getDriveMotorSupplyCurrentAmps();

    inputs.turnAbsolutePosition = moduleSimulation.getTurnAbsolutePosition();
    inputs.turnVelocity = moduleSimulation.getTurnAbsoluteEncoderSpeedRadPerSec();
    inputs.turnAppliedVolts = moduleSimulation.getTurnMotorAppliedVolts();
    inputs.turnCurrentAmps = moduleSimulation.getTurnMotorSupplyCurrentAmps();

    inputs.odometryDriveWheelRevolutions = moduleSimulation.getCachedDriveWheelFinalPositionsRad();

    inputs.odometrySteerPositions = moduleSimulation.getCachedTurnAbsolutePositions();

    inputs.odometryTimestamps = OdometryTimestampsSim.getTimestamps();

    inputs.isConnected = true;
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    moduleSimulation.requestDriveVoltageOut(volts);
  }

  @Override
  public void setTurnVoltage(Voltage volts) {
    moduleSimulation.requestTurnVoltageOut(volts);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        desiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    moduleSimulation.requestDriveVoltageOut(
        Volts.of(
                drivePID.calculate(
                    RadiansPerSecond.of(moduleSimulation.getDriveWheelFinalSpeedRadPerSec())
                            .in(RotationsPerSecond)
                        * ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                    desiredDriveRPS))
            .plus(driveFF.calculate(RotationsPerSecond.of(desiredDriveRPS))));
    moduleSimulation.requestTurnVoltageOut(
        Volts.of(
                turnPID.calculate(
                    moduleSimulation.getTurnAbsolutePosition().getRotations(),
                    desiredState.angle.getRotations()))
            .plus(turnFF.calculate(RotationsPerSecond.of(turnPID.getSetpoint().velocity))));
  }

  @Override
  public double getTurnRotations() {
    return moduleSimulation.getTurnAbsolutePosition().getRotations();
  }

  @Override
  public void stopModule() {
    moduleSimulation.requestDriveVoltageOut(Volts.zero());
    moduleSimulation.requestTurnVoltageOut(Volts.zero());
  }
}
