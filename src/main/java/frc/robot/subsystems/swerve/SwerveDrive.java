package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.extras.Alert;
import frc.robot.extras.TimeUtils;
import frc.robot.extras.VirtualSubsystem;
import frc.robot.subsystems.swerve.gyroIO.GyroIO;
import frc.robot.subsystems.swerve.gyroIO.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.moduleIO.ModuleIO;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import frc.robot.subsystems.swerve.odometryThread.OdometryThreadInputsAutoLogged;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.DriveTrainConstants.*;

public class SwerveDrive extends VirtualSubsystem {
    public enum DriveType {
        GENERIC,
        CTRE_ON_RIO,
        CTRE_ON_CANIVORE
    }

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs;
    private final OdometryThreadInputsAutoLogged odometryThreadInputs;
    private final SwerveModule[] swerveModules;

    private Rotation2d rawGyroRotation;
    private final SwerveModulePosition[] lastModulePositions;
    private final SwerveDrivePoseEstimator odometry;

    private final OdometryThread odometryThread;
    private final Alert gyroDisconnectedAlert = new Alert("Gyro Hardware Fault", Alert.AlertType.ERROR),  visionNoResultAlert = new Alert("Vision No Result", Alert.AlertType.INFO);
    public SwerveDrive(DriveType type, GyroIO gyroIO, ModuleIO frontLeftModuleIO, ModuleIO frontRightModuleIO, ModuleIO backLeftModuleIO, ModuleIO backRightModuleIO) {
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
        this.odometry = new SwerveDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS, rawGyroRotation, lastModulePositions, new Pose2d(),
                VecBuilder.fill(1,2,3), // TODO: add da constants
                VecBuilder.fill(4,5,6) // TODO: add da constants
        );

        this.odometryThread = OdometryThread.createInstance(type);
        this.odometryThreadInputs = new OdometryThreadInputsAutoLogged();
        this.odometryThread.start();

        gyroDisconnectedAlert.setActivated(false);
        visionNoResultAlert.setActivated(false);

        startDashboardDisplay();
    }

    
    public void periodic(double dt, boolean enabled) {
        final double t0 = TimeUtils.getRealTimeSeconds();
        fetchOdometryInputs();
        Logger.recordOutput("SystemPerformance/OdometryFetchingTimeMS", (TimeUtils.getRealTimeSeconds() - t0)*1000);
        modulesPeriodic(dt, enabled);

        for (int timeStampIndex = 0; timeStampIndex < odometryThreadInputs.measurementTimeStamps.length; timeStampIndex++)
            feedSingleOdometryDataToPositionEstimator(timeStampIndex);

        final double timeNotVisionResultSeconds = TimeUtils.getLogTimeSeconds() - previousMeasurementTimeStamp;
        visionNoResultAlert.setText(String.format("AprilTag Vision No Result For %.2f (s)", timeNotVisionResultSeconds));
        visionNoResultAlert.setActivated(timeNotVisionResultSeconds > 4);
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
            module.periodic(dt, enabled);
    }

    private void feedSingleOdometryDataToPositionEstimator(int timeStampIndex) {
        final SwerveModulePosition[] modulePositions = getModulesPosition(timeStampIndex),
                moduleDeltas = getModulesDelta(modulePositions);

        if (!updateRobotFacingWithGyroReading(timeStampIndex))
            updateRobotFacingWithOdometry(moduleDeltas);

        odometry.updateWithTime(
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

    
    public void runRawChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] setpointStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CHASSIS_MAX_VELOCITY);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            optimizedSetpointStates[i] = swerveModules[i].runSetPoint(setpointStates[i]);

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    
    public void stop() {
        Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
        for (int i = 0; i < swerveHeadings.length; i++)
            swerveHeadings[i] = new Rotation2d();
        DriveConstants.DRIVE_KINEMATICS.resetHeadings(swerveHeadings);
        runRobotCentricChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * Locks the chassis and turns the modules to an X formation to resist movement.
     * The lock will be cancelled the next time a nonzero velocity is requested.
     */
    public void lockChassisWithXFormation() {
        Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
        for (int i = 0; i < swerveHeadings.length; i++)
            swerveHeadings[i] = DriveConstants.MODULE_TRANSLATIONS[i].getAngle();
        DriveConstants.DRIVE_KINEMATICS.resetHeadings(swerveHeadings);
        runRobotCentricChassisSpeeds(new ChassisSpeeds());
    }

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
        return odometry.getEstimatedPosition();
    }
    
    public Rotation2d getRawGyroYaw() {return Rotation2d.fromDegrees(gyroInputs.yawPosition); }

    
    public void setPose(Pose2d pose) {
        odometry.resetPosition(rawGyroRotation, getModuleLatestPositions(), pose);
    }

    
    public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public double getChassisMaxLinearVelocityMetersPerSec() {return CHASSIS_MAX_VELOCITY;}
    public double getChassisMaxAccelerationMetersPerSecSq() {return CHASSIS_MAX_ACCELERATION_MPS_SQ;}
    public double getChassisMaxAngularVelocity() {return CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC;}
    public double getChassisMaxAngularAccelerationRadPerSecSq() {return CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ;}

    
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> measurementStdDevs) {
        odometry.addVisionMeasurement(visionPose, timestamp ,measurementStdDevs);
        previousMeasurementTimeStamp = Math.max(timestamp, previousMeasurementTimeStamp);
    }

    private double previousMeasurementTimeStamp = -1;
    
    public double getPreviousVisionMeasurementTimeStamp() {
        return previousMeasurementTimeStamp;
    }

    private void startDashboardDisplay() {
        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Front Right Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Back Left Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Back Right Angle", () -> swerveModules[0].getSteerFacing().getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> swerveModules[0].getDriveVelocityMetersPerSec(), null);

            // builder.addDoubleProperty("Robot Angle", () -> getFacing().minus(FieldConstants.getDriverStationFacing()).getRadians(), null);
        });
    }


    public Rotation2d getFacing() {return getPose().getRotation(); }

    public ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeedsRobotRelative(), getFacing());
    }

    /**
     * runs a driverstation-centric ChassisSpeeds
     * @param driverStationCentricSpeeds a continuous chassis speeds, driverstation-centric, normally from a gamepad
     * */
    public void runDriverStationCentricChassisSpeeds(ChassisSpeeds driverStationCentricSpeeds) {
        final Rotation2d driverStationFacing = DriverStation.getAlliance().equals(Alliance.Blue) ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(Math.PI); 
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                driverStationCentricSpeeds,
                getPose().getRotation().minus(driverStationFacing)
        ));
    }

    /**
     * runs a field-centric ChassisSpeeds
     * @param fieldCentricSpeeds a continuous chassis speeds, field-centric, normally from a pid position controller
     * */
    public void runFieldCentricChassisSpeeds(ChassisSpeeds fieldCentricSpeeds) {
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldCentricSpeeds,
                getPose().getRotation()
        ));
    }

    /**
     * runs a ChassisSpeeds, pre-processed with ChassisSpeeds.discretize()
     * @param speeds a continuous chassis speed, robot-centric
     * */
    public void runRobotCentricChassisSpeeds(ChassisSpeeds speeds) {
        final double PERCENT_DEADBAND = 0.03;
        if (Math.abs(speeds.omegaRadiansPerSecond) < PERCENT_DEADBAND * getChassisMaxAngularVelocity()
            && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < PERCENT_DEADBAND * getChassisMaxLinearVelocityMetersPerSec())
            speeds = new ChassisSpeeds();

        runRawChassisSpeeds(ChassisSpeeds.discretize(speeds, 0.02));
    }

    static boolean isZero(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.omegaRadiansPerSecond) < Math.toRadians(5) && Math.abs(chassisSpeeds.vxMetersPerSecond) < 0.05 && Math.abs(chassisSpeeds.vyMetersPerSecond) < 0.05;
    }

    public ChassisSpeeds constrainAcceleration(
            ChassisSpeeds currentSpeeds, ChassisSpeeds desiredSpeeds,
            double dtSecs) {
        final double
        // TODO: come back to this
                MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = getChassisMaxLinearVelocityMetersPerSec(),
                // / LINEAR_ACCELERATION_SMOOTH_OUT_SECONDS,
                MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = getChassisMaxAngularVelocity();
                // / ANGULAR_ACCELERATION_SMOOTH_OUT_SECONDS;

        Translation2d currentLinearVelocityMetersPerSec = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
                desiredLinearVelocityMetersPerSec = new Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond),
                linearVelocityDifference = desiredLinearVelocityMetersPerSec.minus(currentLinearVelocityMetersPerSec);

        final double maxLinearVelocityChangeIn1Period = MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ * dtSecs;
        final boolean desiredLinearVelocityReachableWithin1Period = linearVelocityDifference.getNorm() <= maxLinearVelocityChangeIn1Period;
        final Translation2d linearVelocityChangeVector = new Translation2d(maxLinearVelocityChangeIn1Period, linearVelocityDifference.getAngle()),
                newLinearVelocity = desiredLinearVelocityReachableWithin1Period ?
                desiredLinearVelocityMetersPerSec
                : currentLinearVelocityMetersPerSec.plus(linearVelocityChangeVector);

        final double angularVelocityDifference = desiredSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond,
                maxAngularVelocityChangeIn1Period = MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ * dtSecs,
                angularVelocityChange = Math.copySign(maxAngularVelocityChangeIn1Period, angularVelocityDifference);
        final boolean desiredAngularVelocityReachableWithin1Period = Math.abs(angularVelocityDifference) <= maxAngularVelocityChangeIn1Period;
        final double newAngularVelocity = desiredAngularVelocityReachableWithin1Period ?
                desiredSpeeds.omegaRadiansPerSecond
                : currentSpeeds.omegaRadiansPerSecond + angularVelocityChange;
        return new ChassisSpeeds(
                newLinearVelocity.getX(),
                newLinearVelocity.getY(),
                newAngularVelocity
        );
    }
}

