// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class ManualIntakeandIndexerRollers extends Command {
  private final Intake intake;
  private final Indexer indexer;
  private boolean direction;

  /** Creates a new ManualIntakeRollers. */
  public ManualIntakeandIndexerRollers(Intake intake, Indexer indexer, boolean direction) {
    this.intake = intake;
    this.indexer = indexer;
    this.direction = direction;
    addRequirements(intake, indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeSpeed(direction ? IntakeConstants.INTAKE_SPEED : -IntakeConstants.INTAKE_SPEED);
    indexer.setIndexerSpeed(
        direction ? IntakeConstants.INTAKE_SPEED : -IntakeConstants.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(IntakeConstants.INTAKE_NEUTRAL_SPEED);
    indexer.setIndexerSpeed(IndexerConstants.INDEXER_NEUTRAL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
