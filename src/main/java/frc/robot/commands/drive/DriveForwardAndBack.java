// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriveForwardAndBack extends Command {
  private final SwerveDrive driveSubsystem;
  Timer timer;

  /** Creates a new DriveForwardAndBack. */
  public DriveForwardAndBack(SwerveDrive driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    timer = new Timer();
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!timer.hasElapsed(2)) {
      driveSubsystem.drive(-0.5, 0, 0, false);
    } else {
      driveSubsystem.drive(0.4, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(5);
  }
}