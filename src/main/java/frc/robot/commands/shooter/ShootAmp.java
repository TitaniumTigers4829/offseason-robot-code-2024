// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelConstants;
import java.util.function.BooleanSupplier;

public class ShootAmp extends Command {
  private final Pivot pivot;
  private final Elevator elevator;
  private final Flywheel flywheel;
  private final BooleanSupplier shoot;

  /** Creates a new ShootAmp. */
  public ShootAmp(Flywheel flywheel, Pivot pivot, Elevator elevator, BooleanSupplier shoot) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.flywheel = flywheel;
    this.shoot = shoot;
    addRequirements(pivot, elevator, flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setElevatorPosition(ElevatorConstants.SHOOT_AMP_POSITION);
    pivot.setPivotAngle(PivotConstants.SHOOT_AMP_ANGLE);
    flywheel.setFlywheelVelocity(FlywheelConstants.SHOOT_AMP_RPM);
    if (shoot.getAsBoolean()) {
      flywheel.setRollerSpeed(FlywheelConstants.ROLLER_SHOOT_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorPosition(ElevatorConstants.INTAKE_POSITION);
    pivot.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    flywheel.setFlywheelVelocity(FlywheelConstants.SHOOTER_NEUTRAL_SPEED);
    flywheel.setRollerSpeed(FlywheelConstants.ROLLER_NEUTRAL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
