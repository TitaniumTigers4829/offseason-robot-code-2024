// All praise 254 lib

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionInterface visionInterface;
  private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

  public Vision(VisionInterface visionInterface) {
    // Initializing Fields
    this.visionInterface = visionInterface;
  }

  @Override
  public void periodic() {
    // Updates limelight inputs
    visionInterface.updateInputs(inputs);
    Logger.processInputs("Vision/", inputs);
  }

  // Add methods to support DriveCommand
  public int getNumberOfAprilTags(Limelight limelight) {
    return visionInterface.getNumberOfAprilTags(limelight);
  }

  public double getLimelightAprilTagDistance(Limelight limelight) {
    return visionInterface.getLimelightAprilTagDistance(limelight);
  }

  public double getTimeStampSeconds(Limelight limelight) {
    return visionInterface.getTimeStampSeconds(limelight);
  }

  public double getLatencySeconds(Limelight limelight) {
    return visionInterface.getLatencySeconds(limelight);
  }

  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    visionInterface.setHeadingInfo(headingDegrees, headingRateDegrees);
  }

  @AutoLogOutput(key = "Vision/Has Targets")
  public boolean canSeeAprilTags(Limelight limelight) {
    return visionInterface.canSeeAprilTags(limelight);
  }

  public Pose2d getPoseFromAprilTags(Limelight limelight) {
    return visionInterface.getPoseFromAprilTags(limelight);
  }

  public Pose2d getLastSeenPose() {
    return visionInterface.getLastSeenPose();
  }
}
