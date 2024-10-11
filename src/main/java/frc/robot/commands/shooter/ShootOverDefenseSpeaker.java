// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.interpolators.SingleLinearInterpolator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Roller;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShootOverDefenseSpeaker extends Command {
  private final SwerveDrive swerveDrive;
  private final Flywheel flywheel;
  private final Roller roller;
  private final Pivot pivot;
  private final Elevator elevator;
  private final Vision vision;

  private DoubleSupplier leftX, leftY;
  private final BooleanSupplier isFieldRelative;
  private final SingleLinearInterpolator speakerOverDefenseAngleLookupValues;

  private double headingError = 0;

  private final ProfiledPIDController turnController =
      new ProfiledPIDController(
          ShooterConstants.AUTO_SHOOT_P,
          ShooterConstants.AUTO_SHOOT_I,
          ShooterConstants.AUTO_SHOOT_D,
          ShooterConstants.AUTO_SHOOT_CONSTRAINTS);

  private boolean isRed = false;
  private double desiredHeading = 0;
  private Translation2d speakerPos;

  /** Creates a new ShootOverDefenseSpeaker. */
  public ShootOverDefenseSpeaker(
      SwerveDrive swerveDrive,
      Flywheel flywheel,
      Roller roller,
      Pivot pivot,
      Elevator elevator,
      Vision vision,
      DoubleSupplier leftX,
      DoubleSupplier leftY,
      BooleanSupplier isFieldRelative) {
    super(swerveDrive, vision);
    this.swerveDrive = swerveDrive;
    this.flywheel = flywheel;
    this.pivot = pivot;
    this.roller = roller;
    this.elevator = elevator;
    this.leftX = leftX;
    this.leftY = leftY;
    this.isFieldRelative = isFieldRelative;
    speakerOverDefenseAngleLookupValues =
        new SingleLinearInterpolator(PivotConstants.SPEAKER_OVER_DEFENSE_PIVOT_POSITION);

    addRequirements(swerveDrive, flywheel, roller, pivot, elevator, vision);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // sets alliance to red
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

    speakerPos =
        isRed
            ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y)
            : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

    // get positions of various things
    Translation2d robotPos = swerveDrive.getPose().getTranslation();
    // distance (for speaker lookups)
    double distance = robotPos.getDistance(speakerPos);
    // arctangent for desired heading
    desiredHeading =
        Math.atan2((robotPos.getY() - speakerPos.getY()), (robotPos.getX() - speakerPos.getX()));

    headingError =
        desiredHeading - swerveDrive.getOdometryAllianceRelativeRotation2d().getRadians();

    double turnOutput = deadband(turnController.calculate(headingError, 0));
    // Gets Angle for Speaker
    double speakerAngle = speakerOverDefenseAngleLookupValues.getLookupValue(distance);

    swerveDrive.drive(
        deadband(leftY.getAsDouble()) * 0.5,
        deadband(leftX.getAsDouble()) * 0.5,
        turnOutput,
        !isFieldRelative.getAsBoolean());

    if (distance > ShooterConstants.SHOOTER_FAR_DISTANCE) {
      flywheel.setFlywheelVelocity(ShooterConstants.SHOOT_SPEAKER_FAR_RPM);
    } else if (distance > ShooterConstants.SHOOTER_DISTANCE) {
      flywheel.setFlywheelVelocity(ShooterConstants.SHOOT_SPEAKER_MEDIUM_RPM);
    } else {
      flywheel.setFlywheelVelocity(ShooterConstants.SHOOT_SPEAKER_RPM);
    }

    pivot.setPivotAngle(speakerAngle);
    elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_OVER_DEFENSE);

    if (isReadyToShoot()) {
      roller.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
    } else {
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setFlywheelVelocity(ShooterConstants.SHOOTER_NEUTRAL_SPEED);
    roller.setRollerSpeed(ShooterConstants.ROLLER_NEUTRAL_SPEED);
    pivot.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    elevator.setElevatorPosition(ElevatorConstants.INTAKE_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isReadyToShoot() {
    // TODO: heading
    return flywheel.isShooterWithinAcceptableError()
        && pivot.isPivotWithinAcceptableError()
        && (Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_RADIANS);
  }

  private double deadband(double val) {
    if (Math.abs(val) < HardwareConstants.DEADBAND_VALUE) {
      return 0.0;
    } else {
      return val;
    }
  }
}
