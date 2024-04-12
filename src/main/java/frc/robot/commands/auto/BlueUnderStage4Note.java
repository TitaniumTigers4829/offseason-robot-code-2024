// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueUnderStage4Note extends SequentialCommandGroup {
  /** Creates a new RedUnderStage4Note. */
  public BlueUnderStage4Note(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d(1.338474238586425, 5.590954303741455, Rotation2d.fromDegrees(0)))),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.8),
      new ParallelCommandGroup(
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(1.4),
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blueunderstage 1", false)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem),
      new ParallelCommandGroup(
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(1.4),
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blueunderstage 2", false)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem),
      new ParallelCommandGroup(
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(1.4),
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blueunderstage 3", false)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem)
    );
  }
}
