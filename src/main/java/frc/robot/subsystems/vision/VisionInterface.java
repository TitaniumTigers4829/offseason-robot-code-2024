package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionInterface {
  @AutoLog
  class VisionInputs {
    public boolean isShooterLimelightConnected = false;
    public boolean isFrontLeftLimelightConnected = false;
    public boolean isFrontRightLimelightConnected = false;

    public Pose2d shooterMegaTag1Pose = new Pose2d();
    public double shooterTagCount = 0.0;
    public Pose2d shooterMegaTag2Pose = new Pose2d();
    public double shooterLatency = 0.0;
    public double shooterTargets = 0.0;
    public Pose2d shooterCameraToTargets = new Pose2d();
    public Pose2d shooterRobotToTargets = new Pose2d();

    public Pose2d frontLeftMegaTag1Pose = new Pose2d();
    public double frontLeftTagCount = 0.0;
    public Pose2d frontLeftMegaTag2Pose = new Pose2d();
    public double frontLeftLatency = 0.0;
    public double frontLeftTargets = 0.0;
    public Pose2d frontLeftCameraToTargets = new Pose2d();
    public Pose2d frontLeftRobotToTargets = new Pose2d();

    public Pose2d frontRightMegaTag1Pose = new Pose2d();
    public double frontRightTagCount = 0.0;
    public Pose2d frontRightMegaTag2Pose = new Pose2d();
    public double frontRightLatency = 0.0;
    public double frontRightTargets = 0.0;
    public Pose2d frontRightCameraToTargets = new Pose2d();
    public Pose2d frontRightRobotToTargets = new Pose2d();

    public double camerasAmount = 0.0;
  }

  default void updateInputs(VisionInputs inputs) {}

  default String getLimelightName(int limelightNumber) {
    return "";
  }

  default double getLatencySeconds(int limelightNumber) {
    return 0.0;
  }

  default double getTimeStampSeconds(int limelightNumber) {
    return 0.0;
  }

  default boolean canSeeAprilTags(int limelightNumber) {
    return false;
  }

  default double getLimelightAprilTagDistance(int limelightNumber) {
    return 0.0;
  }

  default int getNumberOfAprilTags(int limelightNumber) {
    return 0;
  }

  default Pose2d getPoseFromAprilTags(int limelightNumber) {
    return null;
  }

  default void setHeadingInfo(double headingDegrees, double headingRateDegrees) {}
}
