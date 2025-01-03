// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.swerve.SwerveDrive;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class BlueFourNote extends SequentialCommandGroup {
//   /** Creates a new BlueSimpleTwoNote. */
//   public BlueFourNote(
//       SwerveDrive driveSubsystem) {
//     addCommands(
//         new InstantCommand(
//             () ->
//                 driveSubsystem.setPose(
//                     new Pose2d(
//                         1.3980597257614136, 5.493067741394043, Rotation2d.fromRadians(0.0000)))),
//         // new ShootSpeakerAuto(
//         //         driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds)
//         //     .withTimeout(1.5),
//         // new ParallelCommandGroup(
//             new FollowChoreoTrajectory(driveSubsystem, "blue to note 1", false),
//             // new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem,
// leds).withTimeout(2)),
//         // new ShootSpeakerAuto(
//         //         driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds)
//             // .withTimeout(1.5),
//         // new ParallelCommandGroup(
//             new FollowChoreoTrajectory(driveSubsystem, "blue note 1 to 2", false),
//             // new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem,
// leds).withTimeout(2)),
//         // new ShootSpeakerAuto(
//                 // driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds)
//             // .withTimeout(1.7),
//         // new ParallelCommandGroup(
//             new FollowChoreoTrajectory(driveSubsystem, "blue note 2 to 3", false));
//             // new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem,
// leds).withTimeout(3)),
//         // new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem,
// leds).withTimeout(1),
//         // new ShootSpeakerAuto(
//                 // driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds)
//             // .withTimeout(2.2),
//         // new StopShooterAndIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem));
//   }
// }
