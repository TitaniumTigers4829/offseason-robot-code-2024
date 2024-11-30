package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation.WHEEL_GRIP;
import frc.robot.extras.util.DeviceCANBus;
import frc.robot.extras.util.TimeUtil;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.gyroIO.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyroIO.GyroInterface;
import frc.robot.subsystems.swerve.moduleIO.ModuleInterface;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import frc.robot.subsystems.swerve.odometryThread.OdometryThreadInputsAutoLogged;
import frc.robot.subsystems.swerve.setpointGen.AdvancedSwerveModuleState;
import frc.robot.subsystems.swerve.setpointGen.SwerveSetpoint;
import frc.robot.subsystems.swerve.setpointGen.SwerveSetpointGenerator;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
  private final GyroInterface gyroIO;
  private final GyroInputsAutoLogged gyroInputs;
  private final OdometryThreadInputsAutoLogged odometryThreadInputs;
  private final SwerveModule[] swerveModules;

  private Rotation2d rawGyroRotation;
  private final SwerveModulePosition[] lastModulePositions;
  private final SwerveDrivePoseEstimator poseEstimator;

  private SwerveSetpoint setpoint = SwerveSetpoint.zeroed();
  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          DriveConstants.MODULE_TRANSLATIONS,
          DCMotor.getKrakenX60(1).withReduction(ModuleConstants.DRIVE_GEAR_RATIO),
          DCMotor.getFalcon500(1).withReduction(ModuleConstants.TURN_GEAR_RATIO),
          ModuleConstants.DRIVE_STATOR_LIMIT,
          56.0, // TODO: Confirm
          11, // TODO: confirm/get this value
          ModuleConstants.WHEEL_DIAMETER_METERS,
          WHEEL_GRIP.TIRE_WHEEL.cof,
          0.01);

  private final OdometryThread odometryThread;

  private final Alert gyroDisconnectedAlert =
      new Alert("Gyro Hardware Fault", Alert.AlertType.kError);

  // This will stay the same throughout the match. These values are harder to test for and tune, so
  // assume this guess is right.
  private static final Vector<N3> stateStandardDeviations =
      VecBuilder.fill(
          DriveConstants.X_POS_TRUST,
          DriveConstants.Y_POS_TRUST,
          Units.degreesToRadians(DriveConstants.ANGLE_TRUST));

  // This will be changed throughout the match depending on how confident we are that the limelight
  // is right.
  private static final Vector<N3> visionMeasurementStandardDeviations =
      VecBuilder.fill(
          VisionConstants.VISION_X_POS_TRUST,
          VisionConstants.VISION_Y_POS_TRUST,
          Units.degreesToRadians(VisionConstants.VISION_ANGLE_TRUST));

  public SwerveDrive(
      GyroInterface gyroIO,
      ModuleInterface frontLeftModuleIO,
      ModuleInterface frontRightModuleIO,
      ModuleInterface backLeftModuleIO,
      ModuleInterface backRightModuleIO) {
    this.gyroIO = gyroIO;
    this.gyroInputs = new GyroInputsAutoLogged();
    this.rawGyroRotation = new Rotation2d();

    swerveModules =
        new SwerveModule[] {
          new SwerveModule(frontLeftModuleIO, "FrontLeft"),
          new SwerveModule(frontRightModuleIO, "FrontRight"),
          new SwerveModule(backLeftModuleIO, "BackLeft"),
          new SwerveModule(backRightModuleIO, "BackRight")
        };

    lastModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            getRawGyroYaw(),
            lastModulePositions,
            new Pose2d(),
            stateStandardDeviations,
            visionMeasurementStandardDeviations);

    this.odometryThread = OdometryThread.createInstance(DeviceCANBus.RIO);
    this.odometryThreadInputs = new OdometryThreadInputsAutoLogged();
    this.odometryThread.start();

    gyroDisconnectedAlert.set(false);

    // setpoint = SwerveSetpoint.zeroed();
  }

  /**
   * Gets the current velocity of the gyro's yaw
   *
   * @return the yaw velocity
   */
  public double getGyroRate() {
    return gyroInputs.yawVelocity;
  }

  /*
   * Updates the pose estimator with the pose calculated from the april tags. How much it
   * contributes to the pose estimation is set by setPoseEstimatorVisionConfidence.
   *
   * @param visionMeasurement The pose calculated from the april tags
   * @param currentTimeStampSeconds The time stamp in seconds of when the pose from the april tags
   *     was calculated.
   */
  public void addPoseEstimatorVisionMeasurement(
      Pose2d visionMeasurement, double currentTimeStampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
  }

  /**
   * Sets the standard deviations of model states, or how much the april tags contribute to the pose
   * estimation of the robot. Lower numbers equal higher confidence and vice versa.
   *
   * @param xStandardDeviation the x standard deviation in meters
   * @param yStandardDeviation the y standard deviation in meters
   * @param thetaStandardDeviation the theta standard deviation in radians
   */
  public void setPoseEstimatorVisionConfidence(
      double xStandardDeviation, double yStandardDeviation, double thetaStandardDeviation) {
    poseEstimator.setVisionMeasurementStdDevs(
        VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }

  public void periodic() {
    final double t0 = TimeUtil.getRealTimeSeconds();
    fetchOdometryInputs();
    Logger.recordOutput(
        "SystemPerformance/OdometryFetchingTimeMS", (TimeUtil.getRealTimeSeconds() - t0) * 1000);
    modulesPeriodic();
  }

  /**
   * Runs characterization on voltage
   *
   * @param volts voltage to set
   */
  public void runCharacterization(double volts) {
    for (SwerveModule module : swerveModules) {
      module.setVoltage(Volts.of(-volts));
    }
  }

  /** Returns the average drive velocity in rotations/sec. */
  // TODO: fix method
  public double getCharacterizationVelocity() {
    double velocity = 0.0;
    for (SwerveModule module : swerveModules) {
      velocity += module.getCharacterizationVelocity();
    }
    return velocity;
  }

  /** Processes odometry inputs */
  private void fetchOdometryInputs() {
    odometryThread.lockOdometry();
    odometryThread.updateInputs(odometryThreadInputs);
    Logger.processInputs("Drive/OdometryThread", odometryThreadInputs);

    for (SwerveModule module : swerveModules) module.updateOdometryInputs();

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    gyroDisconnectedAlert.set(!gyroInputs.isConnected);

    odometryThread.unlockOdometry();
  }

  private void modulesPeriodic() {
    for (SwerveModule module : swerveModules) module.periodic();
  }

  /**
   * Drives the robot using the joysticks.
   *
   * @param xSpeed Speed of the robot in the x direction, positive being forwards.
   * @param ySpeed Speed of the robot in the y direction, positive being left.
   * @param rotationSpeed Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    ChassisSpeeds desiredSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotationSpeed,
                getPose().getRotation().plus(Rotation2d.fromDegrees(getAllianceAngleOffset())))
            : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    // setpoint = new SwerveSetpoint(getChassisSpeeds(), AdvancedSwerveModuleState.fromBase(getModuleStates()));

    setpoint =
        setpointGenerator.generateSetpoint(setpoint, desiredSpeeds, HardwareConstants.TIMEOUT_S);

    setModuleStates(setpoint.moduleStates());
    Logger.recordOutput("SwerveStates/desired state", setpoint.moduleStates());
  }

  /** Returns 0 degrees if the robot is on the blue alliance, 180 if on the red alliance. */
  public double getAllianceAngleOffset() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    double offset =
        alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? 180.0 : 0.0;
    return offset;
  }

  /**
   * Sets the modules to the specified states.
   *
   * @param desiredStates The desired states for the swerve modules. The order is: frontLeft,
   *     frontRight, backLeft, backRight (should be the same as the kinematics).
   */
  public void setModuleStates(AdvancedSwerveModuleState[] desiredStates) {
    for (int i = 0; i < 4; i ++) {
      swerveModules[i].runSetPoint(AdvancedSwerveModuleState.fromBase(desiredStates[i]));
    }
  }

  /**
   * Updates the pose estimator with the pose calculated from the swerve modules.
   *
   * @param timestampIndex index of the timestamp to sample the pose at
   */
  public void addPoseEstimatorSwerveMeasurement() {
    final SwerveModulePosition[] modulePositions = getModulePositions(),
        moduleDeltas = getModulesDelta(modulePositions);

    if (gyroInputs.isConnected) {
      // rawGyroRotation = gyroInputs.odometryYawPositions[timestampIndex];
      rawGyroRotation = getRawGyroYaw();
    } else {
      Twist2d twist = DriveConstants.DRIVE_KINEMATICS.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    poseEstimator.updateWithTime(
        // odometryThreadInputs.measurementTimeStamps[timestampIndex],
        Timer.getFPGATimestamp(), rawGyroRotation, modulePositions);
  }

  private SwerveModulePosition[] getModulesDelta(SwerveModulePosition[] freshModulesPosition) {
    SwerveModulePosition[] deltas = new SwerveModulePosition[swerveModules.length];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      final double deltaDistanceMeters =
          freshModulesPosition[moduleIndex].distanceMeters
              - lastModulePositions[moduleIndex].distanceMeters;
      deltas[moduleIndex] =
          new SwerveModulePosition(deltaDistanceMeters, freshModulesPosition[moduleIndex].angle);
      lastModulePositions[moduleIndex] = freshModulesPosition[moduleIndex];
    }
    return deltas;
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < 3; i++) states[i] = swerveModules[i].getMeasuredState();
    return states;
  }

  // @AutoLogOutput(key = "SwerveStates/Speeds")
  // private ChassisSpeeds getChassisSpeeds() {
  //   return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  // }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < positions.length; i++) positions[i] = swerveModules[i].getPosition();
    return positions;
  }

  /** Gets the fused pose from the pose estimator. */
  @AutoLogOutput(key = "Odometry/RobotPosition")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Gets the current gyro yaw */
  public Rotation2d getRawGyroYaw() {
    return Rotation2d.fromDegrees(gyroInputs.yawDegrees);
  }

  /**
   * Sets the pose
   *
   * @param pose pose to set
   */
  public void resetPosition(Pose2d pose) {
    // for (int timestampIndex = 0;
    // timestampIndex < odometryThreadInputs.measurementTimeStamps.length;
    // timestampIndex++)
    poseEstimator.resetPosition(getRawGyroYaw(), getModulePositions(), pose);
  }
}
