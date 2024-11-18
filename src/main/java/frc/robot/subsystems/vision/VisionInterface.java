package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.extras.vision.MegatagPoseEstimate;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.littletonrobotics.junction.AutoLog;

public interface VisionInterface {
  @AutoLog
  class VisionInputs {
    public boolean isShooterLimelightConnected;
    public boolean isFrontLeftLimelightConnected;
    public boolean isFrontRightLimelightConnected;

    public MegatagPoseEstimate shooterMegaTag1Pose;
    public double shooterTagCount;
    public MegatagPoseEstimate shooterMegaTag2Pose;
    public double shooterLatency;
    public double shooterTargets;
    public Pose2d shooterCameraToTargets;
    public Pose2d shooterRobotToTargets;
    public Limelight shooter;

    public MegatagPoseEstimate frontLeftMegaTag1Pose;
    public double frontLeftTagCount;
    public MegatagPoseEstimate frontLeftMegaTag2Pose;
    public double frontLeftLatency;
    public double frontLeftTargets;
    public Pose2d frontLeftCameraToTargets;
    public Pose2d frontLeftRobotToTargets;
    public Limelight frontLeft;

    public MegatagPoseEstimate frontRightMegaTag1Pose;
    public double frontRightTagCount;
    public MegatagPoseEstimate frontRightMegaTag2Pose;
    public double frontRightLatency;
    public double frontRightTargets;
    public Pose2d frontRightCameraToTargets;
    public Pose2d frontRightRobotToTargets;
    public Limelight frontRight;

    public double camerasAmount;
  }

  default void updateInputs(VisionInputs inputs) {}

  default String getLimelightName(Limelight limelight) {
    return "";
  }

  default double getLatencySeconds(Limelight limelight) {
    return 0.0;
  }

  default double getTimeStampSeconds(Limelight limelight) {
    return 0.0;
  }

  default boolean canSeeAprilTags(Limelight limelight) {
    return false;
  }

  default double getLimelightAprilTagDistance(Limelight limelight) {
    return 0.0;
  }

  default int getNumberOfAprilTags(Limelight limelight) {
    return 0;
  }

  default Pose2d getPoseFromAprilTags(Limelight limelight) {
    return null;
  }

  default void setHeadingInfo(double headingDegrees, double headingRateDegrees) {}

  default Pose2d getLastSeenPose() {
    return null;
  }
}
