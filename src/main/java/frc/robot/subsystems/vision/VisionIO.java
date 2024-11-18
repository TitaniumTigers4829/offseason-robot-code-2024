package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean cameraConnected = false;
    public double latency = 0.0;
    public double fiducialMarksID = 0.0;

    public int camerasAmount = 0;
    public int targetsCount = 0;
  }

  default void updateInputs(VisionIOInputs inputs) {}

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
