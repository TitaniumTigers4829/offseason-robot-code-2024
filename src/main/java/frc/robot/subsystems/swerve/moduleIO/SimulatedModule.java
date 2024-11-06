package frc.robot.subsystems.swerve.moduleIO;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import java.util.Arrays;

/** Wrapper class around {@link SwerveModuleSimulation} */
public class SimulatedModule implements ModuleInterface {
  private final SwerveModuleSimulation moduleSimulation;

  private final PIDController drivePID = new PIDController(.04, 0, 0);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.0, .0, 0.0);

  private final Constraints turnConstraints =
      new Constraints(
          RadiansPerSecond.of(2 * Math.PI).in(RotationsPerSecond),
          RadiansPerSecondPerSecond.of(4 * Math.PI).in(RotationsPerSecondPerSecond));
  private final ProfiledPIDController turnPID =
      new ProfiledPIDController(Radians.of(35).in(Rotations), 0, 0, turnConstraints);
  private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0.77, 0.75, 0);

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

    inputs.odometryDriveWheelRevolutions =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositionsRad())
            .map(Units::radiansToRotations)
            .toArray();

    inputs.odometrySteerPositions = moduleSimulation.getCachedTurnAbsolutePositions();

    inputs.odometryTimestamps = OdometryTimestampsSim.getTimestamps();

    inputs.isConnected = true;

    SmartDashboard.putNumber(
        "turn absolute position", moduleSimulation.getTurnAbsolutePosition().getDegrees());
    SmartDashboard.putNumber(
        "drive pos",
        Radians.of(moduleSimulation.getDriveEncoderFinalPositionRad()).in(Rotations)
            * ModuleConstants.DRIVE_TO_METERS);
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
    double turnRotations = getTurnRotations();
    // setpoint.cosineScale(Rotation2d.fromRotations(turnRotations));
    desiredState.optimize(Rotation2d.fromRotations(turnRotations));

    
    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        desiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    if (Math.abs(desiredDriveRPS) < 0.01) {
      stopModule();
      return;
    }

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
                        .in(RotationsPerSecond),
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
