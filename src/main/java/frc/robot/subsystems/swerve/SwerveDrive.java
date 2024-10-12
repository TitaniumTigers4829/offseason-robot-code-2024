// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.gyroIO.GyroIO;
import frc.robot.subsystems.swerve.gyroIO.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.moduleIO.ModuleIO;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import frc.robot.subsystems.swerve.odometryThread.OdometryThreadInputsAutoLogged;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.Alert;
import frc.robot.extras.DeviceCANBus;
import frc.robot.extras.util.TimeUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  
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

    private Rotation2d rawGyroRotation;
    private final SwerveModulePosition[] lastModulePositions;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final OdometryThread odometryThread;
    private final Alert gyroDisconnectedAlert = new Alert("Gyro Hardware Fault", Alert.AlertType.ERROR),  visionNoResultAlert = new Alert("Vision No Result", Alert.AlertType.INFO);
    public SwerveDrive(GyroIO gyroIO, ModuleIO frontLeftModuleIO, ModuleIO frontRightModuleIO, ModuleIO backLeftModuleIO, ModuleIO backRightModuleIO) {
        super("Drive");
        this.gyroIO = gyroIO;
        this.gyroInputs = new GyroIOInputsAutoLogged();
        this.rawGyroRotation = new Rotation2d();
        this.swerveModules = new SwerveModule[] {
                new SwerveModule(frontLeftModuleIO, "FrontLeft"),
                new SwerveModule(frontRightModuleIO, "FrontRight"),
                new SwerveModule(backLeftModuleIO, "BackLeft"),
                new SwerveModule(backRightModuleIO, "BackRight"),
        };

        lastModulePositions = new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
        this.poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS, rawGyroRotation, lastModulePositions, new Pose2d(), stateStandardDeviations, visionMeasurementStandardDeviations
        );

        this.odometryThread = OdometryThread.createInstance(DeviceCANBus.CANIVORE);
        this.odometryThreadInputs = new OdometryThreadInputsAutoLogged();
        this.odometryThread.start();

        gyroDisconnectedAlert.setActivated(false);
        visionNoResultAlert.setActivated(false);
    }

    public void periodic(double dt, boolean enabled) {
        final double t0 = TimeUtil.getRealTimeSeconds();
        fetchOdometryInputs();
        Logger.recordOutput("SystemPerformance/OdometryFetchingTimeMS", (TimeUtil.getRealTimeSeconds() - t0)*1000);
        modulesPeriodic(dt, enabled);

        for (int timeStampIndex = 0; timeStampIndex < odometryThreadInputs.measurementTimeStamps.length; timeStampIndex++)
            feedSingleOdometryDataToPositionEstimator(timeStampIndex);

        final double timeNotVisionResultSeconds = TimeUtil.getLogTimeSeconds() - previousMeasurementTimeStamp;
        visionNoResultAlert.setText(String.format("AprilTag Vision No Result For %.2f (s)", timeNotVisionResultSeconds));
        visionNoResultAlert.setActivated(timeNotVisionResultSeconds > 4);
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
      SwerveModuleState[] swerveModuleStates =
          DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
              fieldRelative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed, ySpeed, rotationSpeed, Rotation2d.fromDegrees(0))
                  : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
  
      for (SwerveModuleState swerveModuleState : swerveModuleStates) {
        setModuleStates(swerveModuleState);
      }
      Logger.recordOutput("SwerveStates/SwerveModuleStates", swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState states) {
      for (int i = 0; i < 4; i++) {
        swerveModules[i].runSetPoint(states);
      }
    }

    private void fetchOdometryInputs() {
        odometryThread.lockOdometry();
        odometryThread.updateInputs(odometryThreadInputs);
        Logger.processInputs("Drive/OdometryThread", odometryThreadInputs);

        for (var module : swerveModules)
            module.updateOdometryInputs();

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        gyroDisconnectedAlert.setActivated(!gyroInputs.connected);

        odometryThread.unlockOdometry();
    }

    private void modulesPeriodic(double dt, boolean enabled) {
        for (var module : swerveModules)
            module.periodic();
    }

    private void feedSingleOdometryDataToPositionEstimator(int timeStampIndex) {
        final SwerveModulePosition[] modulePositions = getModulesPosition(timeStampIndex),
                moduleDeltas = getModulesDelta(modulePositions);

        if (!updateRobotFacingWithGyroReading(timeStampIndex))
            updateRobotFacingWithOdometry(moduleDeltas);

        poseEstimator.updateWithTime(
                odometryThreadInputs.measurementTimeStamps[timeStampIndex],
                rawGyroRotation,
                modulePositions
        );
    }

    private SwerveModulePosition[] getModulesPosition(int timeStampIndex) {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++)
            swerveModulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[timeStampIndex];
        return swerveModulePositions;
    }

    private SwerveModulePosition[] getModulesDelta(SwerveModulePosition[] freshModulesPosition) {
        SwerveModulePosition[] deltas = new SwerveModulePosition[swerveModules.length];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            final double deltaDistanceMeters = freshModulesPosition[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters;
            deltas[moduleIndex] = new SwerveModulePosition(deltaDistanceMeters, freshModulesPosition[moduleIndex].angle);
            lastModulePositions[moduleIndex] = freshModulesPosition[moduleIndex];
        }
        return deltas;
    }

    /**
     * updates the robot facing using the reading from the gyro
     * @param timeStampIndex the index of the time stamp
     * @return whether the update is success
     * */
    private boolean updateRobotFacingWithGyroReading(int timeStampIndex) {
        if (!gyroInputs.connected)
            return false;
        rawGyroRotation = gyroInputs.odometryYawPositions[timeStampIndex];
        return true;
    }

    /**
     * updates the robot facing using the reading from the gyro
     * @param modulesDelta the delta of the swerve modules calculated from the odometry
     * */
    private void updateRobotFacingWithOdometry(SwerveModulePosition[] modulesDelta) {
        Twist2d twist = DriveConstants.DRIVE_KINEMATICS.toTwist2d(modulesDelta);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // public void runRawChassisSpeeds(ChassisSpeeds speeds) {
    //     SwerveModuleState[] setpointStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    //     SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, 4.5);

    //     // Send setpoints to modules
    //     SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    //     for (int i = 0; i < 4; i++)
    //         optimizedSetpointStates[i] = swerveModules[i].setDesiredState(setpointStates[i]);

    //     Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    //     Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    // }

    public void stop() {
        Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
        for (int i = 0; i < swerveHeadings.length; i++)
            swerveHeadings[i] = new Rotation2d();
        DriveConstants.DRIVE_KINEMATICS.resetHeadings(swerveHeadings);
    }

    /**
     * Locks the chassis and turns the modules to an X formation to resist movement.
     * The lock will be cancelled the next time a nonzero velocity is requested.
     */
    // public void lockChassisWithXFormation() {
    //     Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
    //     for (int i = 0; i < swerveHeadings.length; i++)
    //         swerveHeadings[i] = MODULE_TRANSLATIONS[i].getAngle();
    //     DriveConstants.DRIVE_KINEMATICS.resetHeadings(swerveHeadings);
    //     HolonomicDriveSubsystem.super.stop();
    // }

    /**
     * Returns the module states (turn angles and drive velocities) for all the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < states.length; i++)
            states[i] = swerveModules[i].getMeasuredState();
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all the modules.
     */
    private SwerveModulePosition[] getModuleLatestPositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < states.length; i++)
            states[i] = swerveModules[i].getLatestPosition();
        return states;
    }

    @AutoLogOutput(key="Odometry/RobotPosition")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    public Rotation2d getRawGyroYaw() {return gyroInputs.yawPosition; }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModuleLatestPositions(), pose);
    }

    public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }
 public double getChassisMaxLinearVelocityMetersPerSec() {return 4.5;} public double getChassisMaxAccelerationMetersPerSecSq() {return 2;} public double getChassisMaxAngularVelocity() {return 3;} public double getChassisMaxAngularAccelerationRadPerSecSq() {return 3;}

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> measurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp ,measurementStdDevs);
        previousMeasurementTimeStamp = Math.max(timestamp, previousMeasurementTimeStamp);
    }

    private double previousMeasurementTimeStamp = -1;
    public double getPreviousVisionMeasurementTimeStamp() {
        return previousMeasurementTimeStamp;
    }

    // private void startDashboardDisplay() {
    //     SmartDashboard.putData("Swerve Drive", builder -> {
    //         builder.setSmartDashboardType("SwerveDrive");

    //         builder.addDoubleProperty("Front Left Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
    //         builder.addDoubleProperty("Front Left Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

    //         builder.addDoubleProperty("Front Right Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
    //         builder.addDoubleProperty("Front Right Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

    //         builder.addDoubleProperty("Back Left Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
    //         builder.addDoubleProperty("Back Left Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

    //         builder.addDoubleProperty("Back Right Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
    //         builder.addDoubleProperty("Back Right Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

    //         builder.addDoubleProperty("Robot Angle", () -> getFacing().minus(FieldConstants.getDriverStationFacing()).getRadians(), null);
    //     });
    // }
}
