package frc.robot.subsystems.swerve.moduleIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

/** Wrapper class around {@link SwerveModuleSimulation} */
public class SimulatedModule implements ModuleInterface {
  private final SwerveModuleSimulation moduleSimulation;

  private final PIDController drivePID = new PIDController(.27, 0, 0);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(1, 1.5);

  private final Constraints turnConstraints =
      new Constraints(
          ModuleConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND,
          ModuleConstants.MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED);
  private final ProfiledPIDController turnPID =
      new ProfiledPIDController(35, 0, 0, turnConstraints);
  private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(4, 10, 0);

  public SimulatedModule(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    inputs.drivePosition =
        Units.radiansToRotations(moduleSimulation.getDriveEncoderFinalPositionRad());
    inputs.driveVelocity =
        Units.radiansToRotations(moduleSimulation.getDriveWheelFinalSpeedRadPerSec());
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
    SmartDashboard.putNumber("desired drive RPS", desiredDriveRPS);
    SmartDashboard.putNumber(
        "current drive RPS",
        RadiansPerSecond.of(moduleSimulation.getDriveWheelFinalSpeedRadPerSec())
            .in(RotationsPerSecond));
    SmartDashboard.putNumber("desired angle", desiredState.angle.getDegrees());

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
    moduleSimulation.requestDriveVoltageOut(Volts.of(0));
    moduleSimulation.requestTurnVoltageOut(Volts.of(0));
  }
}
