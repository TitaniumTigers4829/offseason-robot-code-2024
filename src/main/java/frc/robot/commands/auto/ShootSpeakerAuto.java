// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootSpeakerAuto extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final LEDSubsystem leds;

  private double headingError = 0;

  private final ProfiledPIDController turnController = new ProfiledPIDController(
    ShooterConstants.AUTO_SHOOT_P,
    ShooterConstants.AUTO_SHOOT_I, 
    ShooterConstants.AUTO_SHOOT_D, 
    ShooterConstants.AUTO_SHOOT_CONSTRAINTS
  );

  private boolean isRed = false;
  private double desiredHeading = 0;
  private Translation2d speakerPos;
  private boolean goodToShoot;
  
  /** Creates a new ShootSpeaker. */
  public ShootSpeakerAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, VisionSubsystem visionSubsystem, LEDSubsystem leds, IntakeSubsystem intakeSubsystem) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.leds = leds;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(shooterSubsystem, driveSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    //if alliance is detected
    if (alliance.isPresent()) {
      //and if it's red, we're red
      isRed = alliance.get() == Alliance.Red;
    } else {
      //otherwise default to red alliance
      isRed = true;
    }
    // SmartDashboard.putBoolean("red", isRed);
    speakerPos = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
    // SmartDashboard.putString("speakerPos", speakerPos.toString());
    goodToShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putBoolean("good", goodToShoot);
  
    // get positions of various things
    Translation2d robotPos = driveSubsystem.getPose().getTranslation();
    // distance (for speaker lookups)
    double distance = robotPos.getDistance(speakerPos);
    // arctangent for desired heading
    if (isRed) {
      desiredHeading = Math.atan2((speakerPos.getY() - robotPos.getY()), (speakerPos.getX() - robotPos.getX()));
    } else {
      desiredHeading = Math.atan2((robotPos.getY() - speakerPos.getY()), (robotPos.getX() - speakerPos.getX()));
    }
    // heading error (also used in isReadyToShoot())
    headingError = desiredHeading - driveSubsystem.getPose().getRotation().getRadians();
    // get PID output
    // SmartDashboard.putNumber("desired Heading", desiredHeading);
    // SmartDashboard.putNumber("drivetrain error", headingError);
    // SmartDashboard.putNumber("current heading", driveSubsystem.getRotation2d().getRadians());
    double turnOutput = deadband(turnController.calculate(headingError, 0)); 
    // SmartDashboard.putNumber("turnOutput", turnOutput);
    // SmartDashboard.putNumber("speakerDistance", distance);
    driveSubsystem.drive(
      0, 
      0, 
      turnOutput, 
      false
    );

    if (goodToShoot) {
      shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM);
      pivotSubsystem.setPivotFromDistance(distance);
      // if we are ready to shoot:
      if (isReadyToShoot()) {
        leds.setProcess(LEDProcess.SHOOT);
        shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
      } else {
        leds.setProcess(LEDProcess.FINISH_LINE_UP);
        shooterSubsystem.setRollerSpeed(0);
      }
    } else {
      pivotSubsystem.setPivot(PivotConstants.PIVOT_INTAKE_ANGLE);
      if (pivotSubsystem.isPivotWithinAcceptableError()) {
        if (!shooterSubsystem.getSensor()) {
          leds.setProcess(LEDProcess.INTAKE);
          shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_INTAKE_SPEED);
          intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
        } else {
          goodToShoot = true;
          leds.setProcess(LEDProcess.NOTE_IN);
          intakeSubsystem.setIntakeSpeed(0);
          shooterSubsystem.setRollerSpeed(0);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(0);
    pivotSubsystem.setPivot(PivotConstants.PIVOT_INTAKE_ANGLE);
    leds.setProcess(LEDProcess.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public boolean isReadyToShoot() {
    return shooterSubsystem.isShooterWithinAcceptableError() && pivotSubsystem.isPivotWithinAcceptableError();
  }

  private double deadband(double val) {
    if (Math.abs(val) < 0.1) {
      return 0.0;
    } else {
      return val;
    }
  } 
}
