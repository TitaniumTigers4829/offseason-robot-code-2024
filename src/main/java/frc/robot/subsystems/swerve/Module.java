// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.robot.extras.Alert;
import frc.robot.extras.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP", ModuleConstants.DRIVE_P);
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD",ModuleConstants.DRIVE_D);
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS", ModuleConstants.DRIVE_S);
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV", ModuleConstants.DRIVE_V);  
  private static final LoggedTunableNumber drivekA =
      new LoggedTunableNumber("Drive/Module/DrivekV", ModuleConstants.DRIVE_A);
  private static final LoggedTunableNumber turnkP =
      new LoggedTunableNumber("Drive/Module/TurnkP", ModuleConstants.TURN_P);
  private static final LoggedTunableNumber turnkD =
      new LoggedTunableNumber("Drive/Module/TurnkD", ModuleConstants.TURN_D);
  private static final String[] moduleNames = new String[] {"FL", "FR", "BL", "BR"};

  private final int index;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private SwerveModuleState setpointState = new SwerveModuleState();

  // // Alerts
  // private final Alert driveMotorDisconnected;
  // private final Alert turnMotorDisconnected;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // driveMotorDisconnected =
    //     new Alert(moduleNames[index] + " drive motor disconnected!", Alert.AlertType.WARNING);
    // turnMotorDisconnected =
    //     new Alert(moduleNames[index] + " turn motor disconnected!", Alert.AlertType.WARNING);
  }

  /** Called while blocking odometry thread */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    // Update ff and controllers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setDriveFF(drivekS.get(), drivekV.get(), drivekA.get()),
        drivekS,
        drivekV,
        drivekA);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setDrivePID(drivekP.get(), 0, drivekD.get()), drivekP, drivekD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setTurnPID(turnkP.get(), 0, turnkD.get()), turnkP, turnkD);

    // // Display alerts
    // driveMotorDisconnected.set(!inputs.driveMotorConnected);
    // turnMotorDisconnected.set(!inputs.turnMotorConnected);
  }

  

  /** Sets brake mode to {@code enabled}. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Stops motors. */
  public void stop() {
    io.stop();
  }

  /** Get all latest {@link SwerveModulePosition}'s from last cycle. */
  public SwerveModulePosition[] getModulePositions() {
    int minOdometryPositions =
        Math.min(inputs.odometryDrivePositionsMeters.length, inputs.odometryTurnPositions.length);
    SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
    for (int i = 0; i < minOdometryPositions; i++) {
      positions[i] =
          new SwerveModulePosition(
              inputs.odometryDrivePositionsMeters[i], inputs.odometryTurnPositions[i]);
    }
    return positions;
  }

  /** Get turn angle of module as {@link Rotation2d}. */
  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  /** Get position of wheel rotations in radians */
  public double getPositionRads() {
    return inputs.drivePositionRads;
  }

  /** Get position of wheel in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRads * (ModuleConstants.WHEEL_DIAMETER_METERS * 0.5);
  }

  /** Get velocity of wheel in m/s. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadsPerSec * (ModuleConstants.WHEEL_DIAMETER_METERS * 0.5);
  }

  /** Get current {@link SwerveModulePosition} of module. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Get current {@link SwerveModuleState} of module. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Get velocity of drive wheel for characterization */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadsPerSec;
  }

  public void periodicFunction(){

  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double[] getOdometryPositions() {
    return inputs.odometryDrivePositionsMeters;
  }
}
