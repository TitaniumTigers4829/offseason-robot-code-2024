// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelConstants;

public class SpinupFlywheel extends Command {

  /** Creates a new SpinupFlywheel. */
  private Flywheel flywheelSubsystem;

  public SpinupFlywheel(Flywheel flywheelSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.'
    addRequirements(flywheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheelSubsystem.setFlywheelVelocity(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheelSubsystem.setFlywheelVelocity(FlywheelConstants.FLYWHEEL_SPINUP_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
