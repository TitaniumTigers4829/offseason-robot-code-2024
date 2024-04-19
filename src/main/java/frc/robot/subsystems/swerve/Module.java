package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.ModuleIO.ModuleIOInputs;

public class Module {
  private ModuleIO moduleIO;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputs();
  private int index;

  private double lastPositionMeters = 0.0; // Used for delta calculation
  private SwerveModulePosition positionDelta;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public Module(ModuleIO io, int index) {
    moduleIO = io;
    this.updateInputs();
    this.index = index;
  }

  public void updateInputs() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    double positionMeters = inputs.drivePositionMeters;
    Rotation2d angle = inputs.turnAbsolutePosition; 
    positionDelta = new SwerveModulePosition(inputs.drivePositionMeters - lastPositionMeters, angle);
    lastPositionMeters = positionMeters;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnAbsolutePosition);
  }

  public SwerveModuleState getFieldRelativeState(Rotation2d gyroAngle){
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnAbsolutePosition.plus(gyroAngle));
  }

  public SwerveModuleState getOptimizedState() {
    return moduleIO.getOptimizedState();
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnAbsolutePosition);
  }

  public SwerveModulePosition getPositionDelta() {
    return positionDelta;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    moduleIO.setDesiredState(desiredState);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    moduleIO.resetEncoders();
  }
}