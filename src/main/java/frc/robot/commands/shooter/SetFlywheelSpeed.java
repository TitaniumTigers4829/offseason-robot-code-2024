// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Flywheel;
import java.util.function.DoubleSupplier;

public class SetFlywheelSpeed extends Command {
  private final Flywheel flywheel;
  private DoubleSupplier speed;

  /** Creates a new SetFlywheelSpeed. */
  public SetFlywheelSpeed(Flywheel flywheel, DoubleSupplier speed) {
    this.flywheel = flywheel;
    this.speed = speed;

    addRequirements(flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.setFlywheelVelocity(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
