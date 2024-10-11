// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;

public class IntakeFromGround extends Command {
  private final Intake intake;
  private final Pivot pivot;
  private final Elevator elevator;
  private final Indexer indexer;

  /** Creates a new IntakeFromGround. */
  public IntakeFromGround(Intake intake, Pivot pivot, Elevator elevator, Indexer indexer) {
    this.intake = intake;
    this.pivot = pivot;
    this.elevator = elevator;
    this.indexer = indexer;
    addRequirements(intake, pivot, elevator, indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setElevatorPosition(ElevatorConstants.INTAKE_POSITION);
    pivot.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    intake.setPivotAngle(IntakeConstants.INTAKE_PIVOT_OUT);
    indexer.setIndexerSpeed(ShooterConstants.ROLLER_INTAKE_BEFORE_LATCH_SPEED);
    intake.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPivotAngle(IntakeConstants.INTAKE_PIVOT_IN);
    intake.setIntakeSpeed(IntakeConstants.INTAKE_NEUTRAL_SPEED);
    indexer.setIndexerSpeed(ShooterConstants.ROLLER_NEUTRAL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
