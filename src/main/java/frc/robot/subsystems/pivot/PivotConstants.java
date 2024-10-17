package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

public class PivotConstants {
  public static final double PIVOT_MASS = 0 - 9;
  public static final double PIVOT_LENGTH = 0 - 9;

  public static final double PIVOT_GEAR_RATIO = 8.0;

  public static final int LEADER_PIVOT_MOTOR_ID = 9;
  public static final int FOLLOWER_PIVOT_MOTOR_ID = 10;
  public static final int PIVOT_ENCODER_ID = 33;

  public static final double SUBWOOFER_ANGLE = 0.029;

  public static final double MIN_ANGLE = -0.00341796875;
  public static final double MAX_ANGLE = 0.5537109375;

  public static final double PIVOT_INTAKE_ANGLE = -0.002597265625;

  public static final double PIVOT_P = 160.0;
  public static final double PIVOT_I = 6.0;
  public static final double PIVOT_D = 0.06;
  public static final double PIVOT_G = 0.2;

  public static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 4;
  public static final double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 10;

  public static final double PIVOT_NEUTRAL_SPEED = 0;

  public static final double ANGLE_ZERO = -0.461669921875 - 0.0078125;
  public static final SensorDirectionValue ENCODER_REVERSED =
      SensorDirectionValue.Clockwise_Positive;

  public static final double SHOOT_AMP_ANGLE = 0.35205078125;
  public static final double SHOOT_TRAP_ANGLE = 0;
  public static final double PIVOT_ACCEPTABLE_ERROR = 0.015;

  public static double[][] SPEAKER_PIVOT_POSITION = {
    // Distance, Angle (rotations)
    {1.3, 0.029},
    {1.5, 0.037265625},
    {1.7, 0.0436},
    {1.9, 0.054},
    {2.1, 0.062},
    {2.3, 0.07},
    {2.5, 0.077},
    {2.7, 0.081},
    {2.9, 0.086},
    {3.1, 0.0905},
    {3.3, 0.094},
    {3.5, 0.098},
    {3.7, 0.1},
    {3.9, 0.103},
    {4.1, 0.1055},
    {4.3, 0.1077},
    {4.5, 0.10842},
    {4.7, 0.111},
    {4.9, 0.1135}
  };

  public static double[][] SPEAKER_OVER_DEFENSE_PIVOT_POSITION = {
    // Distance, Angle (rotations)
    {0.0, 0.0},
    {0.0, 0.0}
  };

  public static double[][] PASS_PIVOT_POSITION = {
    // Distance, Angle (rotations)
    {10.680643009839416, 0.037400390625},
    {9.11398136590441, 0.038},
    {0 - 9, 0 - 9},
    {0 - 9, 0 - 9},
  };
}
