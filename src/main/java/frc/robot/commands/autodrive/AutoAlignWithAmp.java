// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

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
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
