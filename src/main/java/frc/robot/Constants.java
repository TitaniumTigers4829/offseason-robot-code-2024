package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants.*;

public final class Constants {

  public class LogPaths {
    public static final String SYSTEM_PERFORMANCE_PATH = "SystemPerformance/";
    public static final String PHYSICS_SIMULATION_PATH = "MaplePhysicsSimulation/";
    public static final String APRIL_TAGS_VISION_PATH = "Vision/AprilTags/";
  }

  public static final class HardwareConstants {

    // public static RobotType getRobot() {
    //   if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
    //     new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
    //         .set(true);
    //     robotType = RobotType.COMPBOT;
    //   }
    //   return robotType;
    // }
    public static final Mode currentMode = Mode.SIM;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }

    public static final double TIMEOUT_S = 0.05;

    public static final double SIGNAL_FREQUENCY = 250;

    public static final String CANIVORE_CAN_BUS_STRING = "canivore 1";
    public static final String RIO_CAN_BUS_STRING = "rio";

    public static final double MIN_FALCON_DEADBAND = 0.0001;

    public static final double DEADBAND_VALUE = 0.05;
  }

  public static final class VisionConstants {

    public static final double VISION_X_POS_TRUST = 0.5; // meters
    public static final double VISION_Y_POS_TRUST = 0.5; // meters
    public static final double VISION_ANGLE_TRUST = Units.degreesToRadians(50); // radians

    public static final int FRAMES_BEFORE_ADDING_VISION_MEASUREMENT = 2;
    public static final double LL3_FOV_MARGIN_OF_ERROR = 27;
    public static final double LL3G_FOV_MARGIN_OF_ERROR = 38;

    public static final String SHOOTER_LIMELIGHT_NAME = "limelight-shooter";
    public static final String FRONT_LEFT_LIMELIGHT_NAME = "limelight-left";
    public static final String FRONT_RIGHT_LIMELIGHT_NAME = "limelight-right";

    public static final double[][] APRIL_TAG_POSITIONS = {
      // {x, y, z, rotation (degrees)}
      {
        Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), 120
      }, // 1
      {
        Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), 120
      }, // 2
      {
        Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), 180
      }, // 3
      {
        Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), 180
      }, // 4
      {
        Units.inchesToMeters(578.77), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38), 270
      }, // 5
      {
        Units.inchesToMeters(72.5), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38), 270
      }, // 6
      {
        -Units.inchesToMeters(1.5), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), 0
      }, // 7
      {
        -Units.inchesToMeters(1.5), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), 0
      }, // 8
      {
        Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), 60
      }, // 9
      {
        Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), 60
      }, // 10
      {
        Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0), 300
      }, // 11
      {
        Units.inchesToMeters(468.69), Units.inchesToMeters(177.1), Units.inchesToMeters(52.0), 60
      }, // 12
      {
        Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0), 180
      }, // 13
      {
        Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0), 0
      }, // 14
      {
        Units.inchesToMeters(182.73), Units.inchesToMeters(177.1), Units.inchesToMeters(52.0), 120
      }, // 15
      {
        Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0), 240
      }, // 16
    };

    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, Units.degreesToRadians(180000)}, // 2
      {1.5, 0.02, 0.02, Units.degreesToRadians(180000)}, // 5
      {3, 1.2, 1.2, Units.degreesToRadians(180000)}, // 25
      {4.5, 5.5, 5.5, Units.degreesToRadians(180000)}, // 90
      {8, 10.0, 10.0, Units.degreesToRadians(180000)} // 180
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, Units.degreesToRadians(180000)}, // 0.5
      {1.5, 0.01, 0.01, Units.degreesToRadians(180000)}, // 0.7
      {3, 0.03, 0.03, Units.degreesToRadians(180000)}, // 4
      {4.5, 0.06, 0.06, Units.degreesToRadians(180000)}, // 30
      {8, 0.5, 0.5, Units.degreesToRadians(180000)}, // 90
      {10, 10.0, 10.0, Units.degreesToRadians(180000)} // 90
    };

    // Note Detection Lookup Table
    public static final double[][] noteDetectionLookupTable = {
      {154.27134704589844, 171.6876983642578, Units.inchesToMeters(0), Units.inchesToMeters(7.5)},
      {178.7086181640625, 170.32672119140625, Units.inchesToMeters(4), Units.inchesToMeters(7.5)},
      {205.19854736328125, 167.13938903808594, Units.inchesToMeters(8), Units.inchesToMeters(7.5)},
      {228.66574096679688, 161.6444854736328, Units.inchesToMeters(12), Units.inchesToMeters(7.5)},
      {246.21170043945312, 157.2861328125, Units.inchesToMeters(16), Units.inchesToMeters(7.5)},
      {259.782958984375, 150.37551879882812, Units.inchesToMeters(20), Units.inchesToMeters(7.5)},
      {272.013427734375, 146.05697631835938, Units.inchesToMeters(24), Units.inchesToMeters(7.5)},
      {155.65867614746094, 137.33753967285156, Units.inchesToMeters(0), Units.inchesToMeters(13.5)},
      {179.44638061523438, 137.10073852539062, Units.inchesToMeters(4), Units.inchesToMeters(13.5)},
      {202.794921875, 136.00408935546875, Units.inchesToMeters(8), Units.inchesToMeters(13.5)},
      {221.5397491455078, 133.6504364013672, Units.inchesToMeters(12), Units.inchesToMeters(13.5)},
      {
        238.55738830566406, 131.46266174316406, Units.inchesToMeters(16), Units.inchesToMeters(13.5)
      },
      {
        251.65420532226562, 128.71676635742188, Units.inchesToMeters(20), Units.inchesToMeters(13.5)
      },
      {262.49658203125, 126.91678619384766, Units.inchesToMeters(24), Units.inchesToMeters(13.5)},
      {155.79156494140625, 110.67668914794922, Units.inchesToMeters(0), Units.inchesToMeters(19.5)},
      {176.4880828857422, 111.09066009521484, Units.inchesToMeters(4), Units.inchesToMeters(19.5)},
      {197.07095336914062, 111.10057067871094, Units.inchesToMeters(8), Units.inchesToMeters(19.5)},
      {213.2230682373047, 111.08323669433594, Units.inchesToMeters(12), Units.inchesToMeters(19.5)},
      {227.31912231445312, 110.8406982421875, Units.inchesToMeters(16), Units.inchesToMeters(19.5)},
      {
        241.66949462890625, 110.21771240234375, Units.inchesToMeters(20), Units.inchesToMeters(19.5)
      },
      {252.04759216308594, 109.3546142578125, Units.inchesToMeters(24), Units.inchesToMeters(19.5)},
      {156.43348693847656, 92.26092529296875, Units.inchesToMeters(0), Units.inchesToMeters(25.5)},
      {173.71957397460938, 92.69190216064453, Units.inchesToMeters(4), Units.inchesToMeters(25.5)},
      {191.1551513671875, 93.2912826538086, Units.inchesToMeters(8), Units.inchesToMeters(25.5)},
      {205.46400451660156, 93.71550750732422, Units.inchesToMeters(12), Units.inchesToMeters(25.5)},
      {220.07833862304688, 94.37662506103516, Units.inchesToMeters(16), Units.inchesToMeters(25.5)},
      {232.0174560546875, 94.9604263305664, Units.inchesToMeters(20), Units.inchesToMeters(25.5)},
      {242.53091430664062, 95.50878143310547, Units.inchesToMeters(24), Units.inchesToMeters(25.5)},
      {156.4423828125, 82.93617248535156, Units.inchesToMeters(0), Units.inchesToMeters(29.5)},
      {172.02281188964844, 83.42843627929688, Units.inchesToMeters(4), Units.inchesToMeters(29.5)},
      {187.8604736328125, 84.26107788085938, Units.inchesToMeters(8), Units.inchesToMeters(29.5)},
      {201.43931579589844, 85.0326156616211, Units.inchesToMeters(12), Units.inchesToMeters(29.5)},
      {214.3050537109375, 85.8529281616211, Units.inchesToMeters(16), Units.inchesToMeters(29.5)},
      {226.25885009765625, 86.8006820678711, Units.inchesToMeters(20), Units.inchesToMeters(29.5)},
      {236.1659698486328, 87.51233673095703, Units.inchesToMeters(24), Units.inchesToMeters(29.5)},
      {156.423828125, 75.72048950195312, Units.inchesToMeters(0), Units.inchesToMeters(33.5)},
      {170.40736389160156, 76.27440643310547, Units.inchesToMeters(4), Units.inchesToMeters(33.5)},
      {184.97666931152344, 77.00422668457031, Units.inchesToMeters(8), Units.inchesToMeters(33.5)},
      {197.5010223388672, 77.55702209472656, Units.inchesToMeters(12), Units.inchesToMeters(33.5)},
      {209.526611328125, 78.64038848876953, Units.inchesToMeters(16), Units.inchesToMeters(33.5)},
      {220.35401916503906, 79.69290161132812, Units.inchesToMeters(20), Units.inchesToMeters(33.5)},
      {230.5920867919922, 80.84618377685547, Units.inchesToMeters(24), Units.inchesToMeters(33.5)},
      {156.3590087890625, 69.85771179199219, Units.inchesToMeters(0), Units.inchesToMeters(37.5)},
      {169.06935119628906, 70.47810363769531, Units.inchesToMeters(4), Units.inchesToMeters(37.5)},
      {182.52525329589844, 71.03211212158203, Units.inchesToMeters(8), Units.inchesToMeters(37.5)},
      {194.46484375, 72.02528381347656, Units.inchesToMeters(12), Units.inchesToMeters(37.5)},
      {205.57937622070312, 72.911865234375, Units.inchesToMeters(16), Units.inchesToMeters(37.5)},
      {215.69342041015625, 73.98934936523438, Units.inchesToMeters(20), Units.inchesToMeters(37.5)},
      {225.42604064941406, 75.15672302246094, Units.inchesToMeters(24), Units.inchesToMeters(37.5)},
      {156.41311645507812, 64.90918731689453, Units.inchesToMeters(0), Units.inchesToMeters(41.5)},
      {168.29574584960938, 65.42889404296875, Units.inchesToMeters(4), Units.inchesToMeters(41.5)},
      {180.56785583496094, 66.30609893798828, Units.inchesToMeters(8), Units.inchesToMeters(41.5)},
      {191.53038024902344, 66.91743469238281, Units.inchesToMeters(12), Units.inchesToMeters(41.5)},
      {202.05491638183594, 68.10053253173828, Units.inchesToMeters(16), Units.inchesToMeters(41.5)},
      {212.24427795410156, 69.29578399658203, Units.inchesToMeters(20), Units.inchesToMeters(41.5)},
      {221.28753662109375, 70.37397766113281, Units.inchesToMeters(24), Units.inchesToMeters(41.5)},
      {156.75836181640625, 60.81660842895508, Units.inchesToMeters(0), Units.inchesToMeters(45.5)},
      {167.243896484375, 61.163150787353516, Units.inchesToMeters(4), Units.inchesToMeters(45.5)},
      {178.677734375, 61.95714569091797, Units.inchesToMeters(8), Units.inchesToMeters(45.5)},
      {188.78147888183594, 62.63084411621094, Units.inchesToMeters(12), Units.inchesToMeters(45.5)},
      {198.72476196289062, 63.67478561401367, Units.inchesToMeters(16), Units.inchesToMeters(45.5)},
      {207.99429321289062, 64.74365234375, Units.inchesToMeters(20), Units.inchesToMeters(45.5)},
      {216.6978759765625, 65.65827941894531, Units.inchesToMeters(24), Units.inchesToMeters(45.5)},
      {156.3396453857422, 57.33953094482422, Units.inchesToMeters(0), Units.inchesToMeters(49.5)},
      {166.42056274414062, 57.79549789428711, Units.inchesToMeters(4), Units.inchesToMeters(49.5)},
      {177.30941772460938, 58.38725280761719, Units.inchesToMeters(8), Units.inchesToMeters(49.5)},
      {186.7626953125, 59.280269622802734, Units.inchesToMeters(12), Units.inchesToMeters(49.5)},
      {
        196.08949279785156, 60.245914459228516, Units.inchesToMeters(16), Units.inchesToMeters(49.5)
      },
      {205.2306671142578, 61.16556930541992, Units.inchesToMeters(20), Units.inchesToMeters(49.5)},
      {213.36785888671875, 62.33674621582031, Units.inchesToMeters(24), Units.inchesToMeters(49.5)},
      {156.1303253173828, 54.62651443481445, Units.inchesToMeters(0), Units.inchesToMeters(53.5)},
      {165.8522491455078, 54.650146484375, Units.inchesToMeters(4), Units.inchesToMeters(53.5)},
      {175.65347290039062, 55.44529724121094, Units.inchesToMeters(8), Units.inchesToMeters(53.5)},
      {184.98977661132812, 56.34360885620117, Units.inchesToMeters(12), Units.inchesToMeters(53.5)},
      {193.63018798828125, 57.18233108520508, Units.inchesToMeters(16), Units.inchesToMeters(53.5)},
      {
        201.44790649414062, 58.028751373291016, Units.inchesToMeters(20), Units.inchesToMeters(53.5)
      },
      {209.90614318847656, 59.1672477722168, Units.inchesToMeters(24), Units.inchesToMeters(53.5)},
    };
  }

  public static final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(653);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(325);

    public static final double RED_AMP_X = 14.82218074798584;
    public static final double RED_AMP_Y = 8.197;

    // TODO: tune
    public static final double RED_AMP_SHOOT_X = 14.82218074798584; // 14.77
    public static final double RED_AMP_SHOOT_Y = 7.774723052978516;

    public static final double BLUE_AMP_X = 1.9;
    public static final double BLUE_AMP_Y = 8.161;

    // TODO: tune
    public static final double BLUE_AMP_SHOOT_X = 1.9;
    public static final double BLUE_AMP_SHOOT_Y = 7.42;

    public static final Rotation2d RED_AMP_ROTATION = Rotation2d.fromDegrees(-90);
    public static final Rotation2d BLUE_AMP_ROTATION = Rotation2d.fromDegrees(-90);

    public static final double RED_SPEAKER_X = 16.511;
    public static final double RED_SPEAKER_Y = 5.55;

    public static final double BLUE_SPEAKER_X = 0;
    public static final double BLUE_SPEAKER_Y = 5.55;

    public static final double RED_LOADING_STATION_X = 1.1;
    public static final double RED_LOADING_STATION_Y = 1.169;

    public static final double BLUE_LOADING_STATION_X = 15.41;
    public static final double BLUE_LOADING_STATION_Y = 1.13;

    // ShootPassing constants
    public static final double RED_PASSING_X = 16.039363861083984;
    public static final double RED_PASSING_Y = 7.130331993103027;

    public static final double BLUE_PASSING_X = 1.343673825263977;
    public static final double BLUE_PASSING_Y = 6.969234943389893;
  }

 

  public static final class ShooterConstants {
    public static final int LEADER_FLYWHEEL_ID = 4;
    public static final int FOLLOWER_FLYWHEEL_ID = 12;
    public static final int ROLLER_MOTOR_ID = 2;

    public static final double SHOOTER_SUPPLY_LIMIT = 60;
    public static final double SHOOTER_STATOR_LIMIT = 60;
    public static final boolean SHOOTER_STATOR_ENABLE = true;
    public static final boolean SHOOTER_SUPPLY_ENABLE = true;

    public static final double ROLLER_NEUTRAL_SPEED = 0;
    public static final double SHOOTER_NEUTRAL_SPEED = 0;

    public static final int NOTE_SENSOR_ID = 4;

    public static final double SHOOT_SPEAKER_RPM = 4000;
    public static final double SHOOT_SPEAKER_FAR_RPM = 4800;
    public static final double SHOOT_SPEAKER_VERY_FAR_RPM = 5500;

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

  public static final class TrajectoryConstants {

    public static final double DRIVE_BASE_DIAMETER =
        Math.sqrt(Math.pow(DriveConstants.TRACK_WIDTH, 2) + Math.pow(DriveConstants.WHEEL_BASE, 2));
    public static final double MAX_SPEED = 5.0;
    public static final double MAX_ACCELERATION = 3;

    public static final double AUTO_TRANSLATION_P = 1.5; // 1.7
    public static final double AUTO_TRANSLATION_D = 0.2;
    public static final double AUTO_THETA_P = 4.5; // 5
    public static final double AUTO_THETA_D = 0.4;

    public static final double AUTO_SHOOT_HEADING_OFFSET = 2;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final double X_TOLERANCE = 0.02;
    public static final double Y_TOLERANCE = 0.02;
    public static final double THETA_TOLERANCE = 1.25;

    // Note Detection Driving Constants
    public static final double AUTO_ALIGN_TRANSLATIONAL_P = 3;
    public static final double AUTO_ALIGN_TRANSLATIONAL_I = 0;
    public static final double AUTO_ALIGN_TRANSLATIONAL_D = 0;

    public static Constraints AUTO_ALIGN_TRANSLATION_CONSTRAINTS = new Constraints(5, 2);

    public static final double AUTO_ALIGN_ROTATIONAL_P = 3;
    public static final double AUTO_ALIGN_ROTATIONAL_I = 0;
    public static final double AUTO_ALIGN_ROTATIONAL_D = 0;

    public static Constraints AUTO_ALIGN_ROTATIONAL_CONSTRAINTS =
        new Constraints(DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 2);
  }

  public static final class JoystickConstants {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;

    public static final int LEFT_STICK_X_ID = 0;
    public static final int LEFT_STICK_Y_ID = 1;
    public static final int RIGHT_STICK_X_ID = 4;

    public static final int A_BUTTON_ID = 1;
    public static final int B_BUTTON_ID = 2;
    public static final int X_BUTTON_ID = 3;
    public static final int Y_BUTTON_ID = 4;

    public static final int LEFT_BUMPER_ID = 5;
    public static final int RIGHT_BUMPER_ID = 6;
    public static final int RIGHT_D_PAD_ID = 90;
    public static final int LEFT_TRIGGER_ID = 2;
    public static final int RIGHT_TRIGGER_ID = 3;
    public static final int RIGHT_STICK_Y_ID = 5;
  }

  public static final class LEDConstants {
    public static final int LED_PORT = 0;

    public static final class SparkConstants {
      // This subclass contains the constant values for the LED patterns.
      public static final double RAINBOW = -0.99;
      public static final double SHOT_RED = -0.85;
      public static final double SHOT_BLUE = -0.83;
      public static final double SHOT_WHITE = -0.81;
      public static final double RED_ALLIANCE_BLINKIN = -0.39;
      public static final double RAINBOW_WAVE = -0.45;
      public static final double OCEAN = -0.41;
      public static final double BOUNCE_RED = -0.35;
      public static final double BOUNCE_GRAY = -0.33;
      public static final double HEARTBEAT_RED = -0.25;
      public static final double HEARTBEAT_GRAY = -0.19;
      public static final double STROBE_RED = -0.11;
      public static final double STROBE_BLUE = -0.09;
      public static final double STROBE_GOLD = -0.07;
      public static final double STROBE_WHITE = -0.05;

      public static final double HEARTBEAT_1 = 0.43;
      public static final double HEARTBEAT_2 = 0.27;

      public static final double MAGENTA = 0.57;
      public static final double DARK_RED = 0.59;
      public static final double RED = 0.61;
      public static final double VERMILION = 0.63;
      public static final double ORANGE = 0.65;
      public static final double GOLD = 0.67;
      public static final double YELLOW = 0.69;
      public static final double LAWN_GREEN = 0.71;
      public static final double LIME = 0.73;
      public static final double DARK_GREEN = 0.75;
      public static final double GREEN = 0.77;
      public static final double CYAN = 0.79;
      public static final double AQUA = 0.81;
      public static final double SKY_BLUE = 0.83;
      public static final double DARK_BLUE = 0.85;
      public static final double BLUE = 0.87;
      public static final double INDIGO = 0.89;
      public static final double PURPLE = 0.91;
      public static final double WHITE = 0.93;
      public static final double GRAY = 0.95;
      public static final double DARK_GRAY = 0.97;
      public static final double BLACK = 0.99;
    }

    public enum LEDProcess {
      /** alliance color */
      ALLIANCE_COLOR(0, 0, 0, 0),
      /** default */
      DEFAULT(0, 0, 0, 0),
      RAINBOW(SparkConstants.RAINBOW, 0, 0, 0),
      RED_ALLIANCE(SparkConstants.RED_ALLIANCE_BLINKIN, 255, 0, 0),
      BLUE_ALLIANCE(SparkConstants.OCEAN, 0, 0, 255),
      SHOOT(SparkConstants.WHITE, 0, 0, 255),
      OFF(SparkConstants.BLACK, 0, 0, 0),
      AUTONOMOUS(SparkConstants.SHOT_WHITE, 0, 0, 0),
      REVERSE_INTAKE(SparkConstants.RED, 0, 255, 0),
      FINISH_LINE_UP(SparkConstants.GREEN, 255, 255, 255),
      GREEN(SparkConstants.GREEN, 0, 255, 0),
      RED(SparkConstants.RED, 255, 0, 0),
      INTAKE(SparkConstants.RED, 255, 0, 0),
      NOTE_IN(SparkConstants.GREEN, 0, 255, 0),
      NOTE_HALFWAY_IN(SparkConstants.YELLOW, 255, 255, 0);

      public final double sparkValue;
      private final int red, green, blue;

      LEDProcess(double sparkValue, int red, int green, int blue) {
        this.sparkValue = sparkValue;
        this.red = red;
        this.green = green;
        this.blue = blue;
      }
    }
  }
}
