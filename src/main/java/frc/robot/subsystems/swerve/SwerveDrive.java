package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.DriveTrainConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.debug.Alert;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.extras.util.DeviceCANBus;
import frc.robot.extras.util.TimeUtil;
import frc.robot.extras.util.VirtualSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.gyroIO.GyroIO;
import frc.robot.subsystems.swerve.gyroIO.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.moduleIO.ModuleIO;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import frc.robot.subsystems.swerve.odometryThread.OdometryThreadInputsAutoLogged;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends VirtualSubsystem {

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

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs;
  private final OdometryThreadInputsAutoLogged odometryThreadInputs;
  private final SwerveModule[] swerveModules;

  private final SwerveModulePosition[] lastModulePositions;
  private final SwerveDrivePoseEstimator odometry;

  private Optional<DriverStation.Alliance> alliance;

  private final OdometryThread odometryThread;
  private final Alert gyroDisconnectedAlert =
      new Alert("Gyro Hardware Fault", Alert.AlertType.ERROR);

  public SwerveDrive(
      GyroIO gyroIO,
      ModuleIO frontLeftModuleIO,
      ModuleIO frontRightModuleIO,
      ModuleIO rearLeftModuleIO,
      ModuleIO rearRightModuleIO) {
    super("Drive");
    this.gyroIO = gyroIO;
    this.gyroInputs = new GyroIOInputsAutoLogged();
    this.swerveModules =
        new SwerveModule[] {
          new SwerveModule(frontLeftModuleIO, "FrontLeft"),
          new SwerveModule(frontRightModuleIO, "FrontRight"),
          new SwerveModule(rearLeftModuleIO, "RearLeft"),
          new SwerveModule(rearRightModuleIO, "RearRight"),
        };

    lastModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    this.odometry =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            getRawGyroYaw(),
            getModuleLatestPositions(),
            new Pose2d(),
            stateStandardDeviations,
            visionMeasurementStandardDeviations);

    this.odometryThread = OdometryThread.createInstance(DeviceCANBus.CANIVORE);
    this.odometryThreadInputs = new OdometryThreadInputsAutoLogged();
    this.odometryThread.start();

    gyroDisconnectedAlert.setActivated(false);

    startDashboardDisplay();
  }

  public void periodic(double dt, boolean enabled) {
    final double t0 = TimeUtil.getRealTimeSeconds();
    fetchOdometryInputs();
    Logger.recordOutput(
        "SystemPerformance/OdometryFetchingTimeMS", (TimeUtil.getRealTimeSeconds() - t0) * 1000);
    modulesPeriodic(dt, enabled);

    for (int timeStampIndex = 0;
        timeStampIndex < odometryThreadInputs.measurementTimeStamps.length;
        timeStampIndex++) updateOdometry(timeStampIndex);
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotationSpeed, getOdometryAllianceRelativeRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    for (SwerveModuleState swerveModuleState : swerveModuleStates) {
      setModuleStates(swerveModuleState);
    }
    Logger.recordOutput("SwerveStates/SwerveModuleStates", swerveModuleStates);
  }

  /**
   * Returns a Rotation2d for the heading of the robot relative to the field from the driver's
   * perspective. This method is needed so that the drive command and poseEstimator don't fight each
   * other. It uses odometry rotation.
   */
  public Rotation2d getOdometryAllianceRelativeRotation2d() {
    if (AllianceFlipper.isBlue()) {
      return getOdometryRotation();
    }
    return AllianceFlipper.flipRotation(getOdometryRotation());
  }

  /**
   * Sets the modules to the specified states.
   *
   * @param desiredStates The desired states for the swerve modules. The order is: frontLeft,
   *     frontRight, backLeft, backRight (should be the same as the kinematics).
   */
  public void setModuleStates(SwerveModuleState desiredStates) {
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates);
    }
  }

  private void fetchOdometryInputs() {
    odometryThread.lockOdometry();
    odometryThread.updateInputs(odometryThreadInputs);
    Logger.processInputs("Drive/OdometryThread", odometryThreadInputs);

    for (var module : swerveModules) module.updateOdometryInputs();

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    gyroDisconnectedAlert.setActivated(!gyroInputs.isConnected);

    odometryThread.unlockOdometry();
  }

  private void modulesPeriodic(double dt, boolean enabled) {
    for (SwerveModule module : swerveModules) module.periodic(dt, enabled);
  }

  private void updateOdometry(int timeStampIndex) {
    final SwerveModulePosition[] modulePositions = getModulesPosition(timeStampIndex),
        moduleDeltas = getModulesDelta(modulePositions);

    if (!isGyroConnected(timeStampIndex)) updateRotationWithOdometry(moduleDeltas);

    odometry.updateWithTime(
        odometryThreadInputs.measurementTimeStamps[timeStampIndex],
        getRawGyroYaw(),
        modulePositions);
  }

  private SwerveModulePosition[] getModulesPosition(int timeStampIndex) {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++)
      swerveModulePositions[moduleIndex] =
          swerveModules[moduleIndex].getOdometryPositions()[timeStampIndex];
    return swerveModulePositions;
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

  /**
   * updates the robot facing using the reading from the gyro
   *
   * @param timeStampIndex the index of the time stamp
   * @return whether the update is success
   */
  private boolean isGyroConnected(int timeStampIndex) {
    return gyroInputs.isConnected;
  }

  /**
   * updates the robot facing using the reading from the gyro
   *
   * @param modulesDelta the delta of the swerve modules calculated from the odometry
   */
  private void updateRotationWithOdometry(SwerveModulePosition[] modulesDelta) {
    Twist2d twist = DriveConstants.DRIVE_KINEMATICS.toTwist2d(modulesDelta);
    getRawGyroYaw().plus(new Rotation2d(twist.dtheta));
  }

  /**
   * Locks the chassis and turns the modules to an X formation to resist movement. The lock will be
   * cancelled the next time a nonzero velocity is requested.
   */
  public void lockChassisWithXFormation() {
    Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
    for (int i = 0; i < swerveHeadings.length; i++)
      swerveHeadings[i] = DriveConstants.MODULE_TRANSLATIONS[i].getAngle();
    DriveConstants.DRIVE_KINEMATICS.resetHeadings(swerveHeadings);
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.stopModule();
    }
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < states.length; i++) states[i] = swerveModules[i].getMeasuredState();
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  private SwerveModulePosition[] getModuleLatestPositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < states.length; i++) states[i] = swerveModules[i].getLatestPosition();
    return states;
  }

  @AutoLogOutput(key = "Odometry/RobotPosition")
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public Rotation2d getRawGyroYaw() {
    return gyroInputs.yawDegrees;
  }

  public void setPose(Pose2d pose) {
    odometry.resetPosition(getRawGyroYaw(), getModuleLatestPositions(), pose);
  }

  public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public double getChassisMaxLinearVelocityMetersPerSec() {
    return CHASSIS_MAX_VELOCITY;
  }

  public double getChassisMaxAccelerationMetersPerSecSq() {
    return CHASSIS_MAX_ACCELERATION_MPS_SQ;
  }

  public double getChassisMaxAngularVelocity() {
    return CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC;
  }

  public double getChassisMaxAngularAccelerationRadPerSecSq() {
    return CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ;
  }

  public void addVisionMeasurement(
      Pose2d visionPose, double timestamp, Matrix<N3, N1> measurementStdDevs) {
    odometry.addVisionMeasurement(visionPose, timestamp, measurementStdDevs);
    previousMeasurementTimeStamp = Math.max(timestamp, previousMeasurementTimeStamp);
  }

  private double previousMeasurementTimeStamp = -1;

  public double getPreviousVisionMeasurementTimeStamp() {
    return previousMeasurementTimeStamp;
  }

  private void startDashboardDisplay() {
    SmartDashboard.putData(
        "Swerve Drive",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> swerveModules[0].getTurnRotation().getRadians(), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> swerveModules[0].getDriveVelocity(), null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> swerveModules[0].getTurnRotation().getRadians(), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () -> swerveModules[0].getDriveVelocity(), null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> swerveModules[0].getTurnRotation().getRadians(), null);
          builder.addDoubleProperty(
              "Back Left Velocity", () -> swerveModules[0].getDriveVelocity(), null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> swerveModules[0].getTurnRotation().getRadians(), null);
          builder.addDoubleProperty(
              "Back Right Velocity", () -> swerveModules[0].getDriveVelocity(), null);

          builder.addDoubleProperty("Robot Angle", () -> getOdometryRotation().getDegrees(), null);
        });
  }

  public Rotation2d getOdometryRotation() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getMeasuredChassisSpeedsRobotRelative(), getOdometryRotation());
  }

  public ChassisSpeeds constrainAcceleration(
      ChassisSpeeds currentSpeeds, ChassisSpeeds desiredSpeeds, double dtSecs) {
    final double
        // TODO: come back to this
        MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = getChassisMaxLinearVelocityMetersPerSec(),
        // / LINEAR_ACCELERATION_SMOOTH_OUT_SECONDS,
        MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = getChassisMaxAngularVelocity();
    // / ANGULAR_ACCELERATION_SMOOTH_OUT_SECONDS;

    Translation2d
        currentLinearVelocityMetersPerSec =
            new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
        desiredLinearVelocityMetersPerSec =
            new Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond),
        linearVelocityDifference =
            desiredLinearVelocityMetersPerSec.minus(currentLinearVelocityMetersPerSec);

    final double maxLinearVelocityChangeIn1Period =
        MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ * dtSecs;
    final boolean desiredLinearVelocityReachableWithin1Period =
        linearVelocityDifference.getNorm() <= maxLinearVelocityChangeIn1Period;
    final Translation2d
        linearVelocityChangeVector =
            new Translation2d(
                maxLinearVelocityChangeIn1Period, linearVelocityDifference.getAngle()),
        newLinearVelocity =
            desiredLinearVelocityReachableWithin1Period
                ? desiredLinearVelocityMetersPerSec
                : currentLinearVelocityMetersPerSec.plus(linearVelocityChangeVector);

    final double
        angularVelocityDifference =
            desiredSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond,
        maxAngularVelocityChangeIn1Period = MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ * dtSecs,
        angularVelocityChange =
            Math.copySign(maxAngularVelocityChangeIn1Period, angularVelocityDifference);
    final boolean desiredAngularVelocityReachableWithin1Period =
        Math.abs(angularVelocityDifference) <= maxAngularVelocityChangeIn1Period;
    final double newAngularVelocity =
        desiredAngularVelocityReachableWithin1Period
            ? desiredSpeeds.omegaRadiansPerSecond
            : currentSpeeds.omegaRadiansPerSecond + angularVelocityChange;
    return new ChassisSpeeds(
        newLinearVelocity.getX(), newLinearVelocity.getY(), newAngularVelocity);
  }
}
