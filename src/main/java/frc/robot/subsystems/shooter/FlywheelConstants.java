package frc.robot.subsystems.shooter;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;

/** All of the constants called in the shooter subsystems. */
public class FlywheelConstants {
  /** ID of the leader flywheel */
  public static final int LEADER_FLYWHEEL_ID = 4;

  /** ID of the follower flywheel */
  public static final int FOLLOWER_FLYWHEEL_ID = 12;

  /** ID of the roller motor */
  public static final int ROLLER_MOTOR_ID = 2;

  /** Sets the limit for the shooter supply voltage */
  public static final double SHOOTER_SUPPLY_LIMIT = 60;

  /** Sets the current limit for the motor stator */
  public static final double SHOOTER_STATOR_LIMIT = 60;

  /** Enables the limit for stator */
  public static final boolean SHOOTER_STATOR_ENABLE = true;

  /** Enables the limit for current to motor */
  public static final boolean SHOOTER_SUPPLY_ENABLE = true;

  /** Sets the speed of the roller motor for neutral mode */
  public static final double ROLLER_NEUTRAL_SPEED = 0;

  /** Sets the speed of the shooter motor for neutral mode */
  public static final double SHOOTER_NEUTRAL_SPEED = 0;

  public static final double FLYWHEEL_SPINUP_SPEED = 4000;
  public static final double SHOOT_SPEAKER_RPM = 4000;
  public static final double SHOOT_SPEAKER_FAR_RPM = 4800;
  public static final double SHOOT_SPEAKER_VERY_FAR_RPM = 5500;

  public static final int SHOOTER_ACCEPTABLE_RPM_ERROR = 25;

  /** Forced Labor has determined that this is the best P value */
  public static final double SHOOT_P = 0.522;

  /** Forced Labor has determined that this is the best I value */
  public static final double SHOOT_I = 0.00;

  /** Forced Labor has determined that this is the best D value */
  public static final double SHOOT_D = 0.001;

  /** Forced Labor has determined that this is the best S value */
  public static final double SHOOT_S = 0.319692618511411;

  /** Forced Labor has determined that this is the best V value */
  public static final double SHOOT_V = 0.125930273774783;

  /** Forced Labor has determined that this is the best A value */
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

  public static final int LEFT_FLYWHEEL_MOTOR_ID = 4;
  public static final int RIGHT_FLYWHEEL_MOTOR_ID = 12;
  public static final int NOTE_SENSOR_ID = 0 - 9;

  public static final double GEAR_RATIO = 1.0;

  public static final double FLYWHEEL_P = 0.0;
  public static final double FLYWHEEL_I = 0.0;
  public static final double FLYWHEEL_D = 0.0;
  public static final double FLYWHEEL_S = 0.0;
  public static final double FLYWHEEL_V = 0.0;
  public static final double FLYWHEEL_A = 0.0;

  public static final double FLYWHEEL_SUPPLY_LIMIT = 60.0;
  public static final boolean FLYWHEEL_SUPPLY_ENABLE = true;

  /**
   * The stator limit limits the current sent to the stator in the motor
   * https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
   */
  public static final double FLYWHEEL_STATOR_LIMIT = 60.0;

  /**
   * Enables the stator limit which limits current to the stator in the motor
   * https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
   */
  public static final boolean FLYWHEEL_STATOR_ENABLE = true;

  public static final double LOW_VOLTAGE_BOUNDARY = 12.0;
  public static final double RPM_RPS_CONVERSION_FACTOR = 60.0;
  public static final double HIGH_VOLTAGE_BOUNDARY = -12.0;
}
