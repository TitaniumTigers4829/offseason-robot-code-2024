// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.*;

public class ManualIntake extends Command {

  private final Intake intake;
  private boolean direction;

  // private final Shooter shooter;

  /** Creates a new ManualPickup. */
  public ManualIntake(Intake intake, boolean direction) {
    this.intake = intake;
    this.direction = direction;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPivotAngle(IntakeConstants.INTAKE_PIVOT_OUT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeSpeed(direction ? IntakeConstants.INTAKE_SPEED : -IntakeConstants.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
    // intake.setPivotAngle(IntakeConstants.INTAKE_PIVOT_IN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
