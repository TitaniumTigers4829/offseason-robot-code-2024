// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;

public class AutoAlignWithAmp extends Command {
  private final SwerveDrive swerveDrive;

  private boolean isRed;
  private Pose2d ampPose;

  private final ProfiledPIDController turnController =
      new ProfiledPIDController(
          ShooterConstants.AUTO_LINEUP_ROTATION_P,
          ShooterConstants.AUTO_LINEUP_ROTATION_I,
          ShooterConstants.AUTO_LINEUP_ROTATION_D,
          ShooterConstants.AUTO_LINEUP_ROTATION_CONSTRAINTS);

  private final ProfiledPIDController xTranslationController =
      new ProfiledPIDController(
          ShooterConstants.AUTO_LINEUP_TRANSLATION_P,
          ShooterConstants.AUTO_LINEUP_TRANSLATION_I,
          ShooterConstants.AUTO_LINEUP_TRANSLATION_D,
          ShooterConstants.AUTO_LINEUP_TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController yTranslationController =
      new ProfiledPIDController(
          ShooterConstants.AUTO_LINEUP_TRANSLATION_P,
          ShooterConstants.AUTO_LINEUP_TRANSLATION_I,
          ShooterConstants.AUTO_LINEUP_TRANSLATION_D,
          ShooterConstants.AUTO_LINEUP_TRANSLATION_CONSTRAINTS);

  /** Creates a new AutoAlignWithAmp. */
  public AutoAlignWithAmp(SwerveDrive swerveDrive, Vision vision) {
    super(swerveDrive, vision);
    this.swerveDrive = swerveDrive;

    addRequirements(swerveDrive, vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // This will default to blue if it alliance isn't present
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    ampPose =
        isRed
            ? new Pose2d(
                FieldConstants.RED_AMP_SHOOT_X,
                FieldConstants.RED_AMP_SHOOT_Y,
                FieldConstants.RED_AMP_ROTATION)
            : new Pose2d(
                FieldConstants.BLUE_AMP_SHOOT_X,
                FieldConstants.BLUE_AMP_SHOOT_Y,
                FieldConstants.BLUE_AMP_ROTATION);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

    // Gets the error between the desired pos (the amp) and the current pos of the robot
    Pose2d drivePose = swerveDrive.getPose();
    double xPoseError = ampPose.getX() - drivePose.getX();
    double yPoseError = ampPose.getY() - drivePose.getY();
    double thetaPoseError =
        ampPose.getRotation().getRadians() - drivePose.getRotation().getRadians();
    // Uses the PID controllers to calculate the drive output
    double xOutput = deadband(xTranslationController.calculate(xPoseError, 0));
    double yOutput = deadband(yTranslationController.calculate(yPoseError, 0));
    double turnOutput = deadband(turnController.calculate(thetaPoseError, 0));

    // Gets the chassis speeds for the robot using the odometry rotation (not alliance relative)
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput, yOutput, turnOutput, swerveDrive.getOdometryAllianceRelativeRotation2d());

    // Drives the robot towards the amp
    swerveDrive.drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double deadband(double val) {
    if (Math.abs(val) < HardwareConstants.DEADBAND_VALUE) {
      return 0.0;
    } else {
      return val;
    }
  }
}
