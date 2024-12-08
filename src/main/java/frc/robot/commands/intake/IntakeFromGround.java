// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelConstants;

public class IntakeFromGround extends Command {
  private final Intake intake;
  private final Pivot pivot;
  private final Elevator elevator;
  private final Flywheel flywheel;
  private final Indexer indexer;

  /** Creates a new IntakeFromGround. */
  public IntakeFromGround(
      Intake intake, Flywheel flywheel, Pivot pivot, Elevator elevator, Indexer indexer) {
    this.intake = intake;
    this.pivot = pivot;
    this.elevator = elevator;
    this.flywheel = flywheel;
    this.indexer = indexer;
    addRequirements(intake, pivot, flywheel, elevator, indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set the elevator and pivot to intake position
    elevator.setElevatorPosition(ElevatorConstants.INTAKE_POSITION);
    pivot.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);

    // If there is no note in the robot extend the otb and run the rollers
    if (!flywheel.hasNote()) {
      intake.setPivotAngle(IntakeConstants.INTAKE_PIVOT_OUT);
      indexer.setIndexerSpeed(IndexerConstants.INTAKE_SPEED);
      intake.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Ends once the note is in the shooter
    // Sets roller, intake, and indexer speeds to zero
    flywheel.setRollerSpeed(FlywheelConstants.ROLLER_NEUTRAL_SPEED);
    intake.setIntakeSpeed(IntakeConstants.INTAKE_NEUTRAL_SPEED);
    indexer.setIndexerSpeed(IndexerConstants.INDEXER_NEUTRAL_SPEED);
    // Puts the intake back in the robot
    intake.setPivotAngle(IntakeConstants.INTAKE_PIVOT_IN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Returns whether or not the shooter has a note
    return flywheel.hasNote();
  }
}
