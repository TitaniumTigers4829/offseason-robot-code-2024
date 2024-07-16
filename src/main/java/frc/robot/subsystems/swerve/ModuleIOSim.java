package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ModuleConstants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim driveSim = new DCMotorSim(DCMotor.getKrakenX60(1), 6.75, 0.025);
  private DCMotorSim turnSim = new DCMotorSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004);

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private PIDController driveFeedback = new PIDController(0,0,0);
  private PIDController turnFeedback = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 0, 0);

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionMeters = driveSim.getAngularPositionRad();
    inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionMeters};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedback.setPID(kP, kI, kD);
  }

  // @Override
  // public void setDriveFF(double kS, double kV, double kA) {
  //   driveFF
  // }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    double turnRadians = getTurnRadians();
 
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnRadians));
    
        if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01) {
          driveSim.setInputVoltage(0);
          turnSim.setInputVoltage(0);
          return;
        }
    
        // Converts meters per second to rotations per second
        double desiredDriveRPM = optimizedDesiredState.speedMetersPerSecond / 60 
         * ModuleConstants.DRIVE_GEAR_RATIO / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
         
         setDriveVoltage(
          driveFeedback.calculate(driveSim.getAngularVelocityRPM(), desiredDriveRPM)
              // + feedForward
              );
        setTurnVoltage(turnFeedback.calculate(turnSim.getAngularPositionRotations(), optimizedDesiredState.angle.getRotations()));
  }
 
  public double getTurnRadians() {
    return turnSim.getAngularPositionRad();
  }
}