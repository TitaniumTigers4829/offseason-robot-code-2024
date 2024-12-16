// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelConstants;

public class ShootSubwoofer extends Command {
  private final Pivot pivot;
  private final Flywheel flywheel;

  /** Creates a new ShootSubwoofer. */
  public ShootSubwoofer(Pivot pivot, Flywheel flywheel) {
    this.pivot = pivot;
    this.flywheel = flywheel;
    addRequirements(pivot, flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.setFlywheelVelocity(FlywheelConstants.SHOOT_SPEAKER_RPM);
    pivot.setPivotAngle(PivotConstants.SUBWOOFER_ANGLE);
    if (pivot.isPivotWithinAcceptableError()) {
      flywheel.setRollerSpeed(FlywheelConstants.ROLLER_SHOOT_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setFlywheelVelocity(FlywheelConstants.SHOOTER_NEUTRAL_SPEED);
    pivot.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    flywheel.setRollerSpeed(FlywheelConstants.ROLLER_NEUTRAL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
