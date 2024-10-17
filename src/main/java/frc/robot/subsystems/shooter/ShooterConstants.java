// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;

/** Add your docs here. */
public final class ShooterConstants {
  public static final int LEADER_FLYWHEEL_ID = 4;
  public static final int FOLLOWER_FLYWHEEL_ID = 12;
  public static final int ROLLER_MOTOR_ID = 2;
  
  public static final double SHOOTER_GEARING = 1;
  public static final double SHOOTER_SUPPLY_LIMIT = 60;
  public static final double SHOOTER_STATOR_LIMIT = 60;
  public static final boolean SHOOTER_STATOR_ENABLE = true;
  public static final boolean SHOOTER_SUPPLY_ENABLE = true;

  public static final double ROLLER_NEUTRAL_SPEED = 0;
  public static final double SHOOTER_NEUTRAL_SPEED = 0;

  public static final int NOTE_SENSOR_ID = 4;

  public static final double SHOOT_SPEAKER_RPM = 4000;
  public static final double SHOOT_SPEAKER_MEDIUM_RPM = 4400;
  public static final double SHOOT_SPEAKER_FAR_RPM = 4800;
  public static final double SHOOT_SPEAKER_VERY_FAR_RPM = 5500;

  public static final double SHOOTER_DISTANCE = 1.8;
  public static final double SHOOTER_FAR_DISTANCE = 3.2;

  public static final int SHOOTER_ACCEPTABLE_RPM_ERROR = 25;

  public static final double SHOOT_P = 0.522;
  public static final double SHOOT_I = 0.00;
  public static final double SHOOT_D = 0.001;
  public static final double SHOOT_S = 0.319692618511411;
  public static final double SHOOT_V = 0.125930273774783;
  public static final double SHOOT_A = 0.004358865417933;

  public static final double ROLLER_SHOOT_SPEED = 1;
  public static final double ROLLER_INTAKE_BEFORE_LATCH_SPEED = .1;
  public static final double SHOOT_AMP_RPM = 2000;

  public static final double AUTO_SHOOT_P = 5; // 7 --> 4.5 --> 5
  public static final double AUTO_SHOOT_I = 0.0;
  public static final double AUTO_SHOOT_D = 0.0;
  public static Constraints AUTO_SHOOT_CONSTRAINTS =
      new Constraints(DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 2);

  public static final double AUTO_SHOOT_MOVE_P = 10.0;
  public static final double AUTO_SHOOT_MOVE_I = 0.0;
  public static final double AUTO_SHOOT_MOVE_D = 0.0;
  public static Constraints AUTO_SHOOT_MOVE_CONSTRAINTS = new Constraints(10, 5);

  public static final double AUTO_LINEUP_ROTATION_P = 3;
  public static final double AUTO_LINEUP_ROTATION_I = 0.0;
  public static final double AUTO_LINEUP_ROTATION_D = 0.0;
  public static Constraints AUTO_LINEUP_ROTATION_CONSTRAINTS =
      new Constraints(DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 2);

  public static final double AUTO_LINEUP_TRANSLATION_P = 10; // 4.5, 4.0
  public static final double AUTO_LINEUP_TRANSLATION_I = 0.0;
  public static final double AUTO_LINEUP_TRANSLATION_D = 0.0;
  public static Constraints AUTO_LINEUP_TRANSLATION_CONSTRAINTS = new Constraints(1, 1);

  // TODO: calc
  public static final double NOTE_LAUNCH_VELOCITY_METERS_PER_SECOND = 10.8;

  public static final double SHOOTER_HEIGHT = Units.inchesToMeters(28.5);
  public static final double SPEAKER_HEIGHT = Units.inchesToMeters(80);
}
