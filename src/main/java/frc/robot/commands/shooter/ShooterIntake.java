// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Flywheel;

public class ShooterIntake extends Command {
  Flywheel flywheel;
  /** Creates a new ShooterIntake. */
  public ShooterIntake(Flywheel flywheel) {
    this.flywheel = flywheel;
    addRequirements(flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.setFlywheelVelocity(-2000);
    flywheel.setRollerSpeed(-0.5);
    if (flywheel.rollerHasNote()) {
      flywheel.setRollerSpeed(0.0);
      flywheel.setFlywheelVelocity(0);
    } 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setFlywheelVelocity(0);
    flywheel.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
