// TigerLib 2024

package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extras.interpolators.MultiLinearInterpolator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.Limelight;

public abstract class DriveCommandBase extends Command {
  private final MultiLinearInterpolator oneAprilTagLookupTable =
      new MultiLinearInterpolator(VisionConstants.ONE_APRIL_TAG_LOOKUP_TABLE);
  private final MultiLinearInterpolator twoAprilTagLookupTable =
      new MultiLinearInterpolator(VisionConstants.TWO_APRIL_TAG_LOOKUP_TABLE);

  private final Vision vision;
  private final SwerveDrive swerveDrive;

  private double lastTimeStampSeconds = 0;

  /**
   * An abstract class that handles pose estimation while driving.
   *
   * @param swerveDrive The subsystem for the swerve drive
   * @param vision The subsystem for vision measurements
   */
  public DriveCommandBase(SwerveDrive swerveDrive, Vision vision) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    // It is important that you do addRequirements(swerveDrive, vision) in whatever
    // command extends this
    // DO NOT do addRequirements here, it will break things
  }

  @Override
  public void execute() {
    swerveDrive.addPoseEstimatorSwerveMeasurement();
    vision.setHeadingInfo(
        swerveDrive.getPose().getRotation().getDegrees(), swerveDrive.getGyroRate());
    calculatePoseFromLimelight(Limelight.SHOOTER);
    calculatePoseFromLimelight(Limelight.FRONT_LEFT);
    calculatePoseFromLimelight(Limelight.FRONT_RIGHT);
  }

  public void calculatePoseFromLimelight(Limelight limelight) {
    double currentTimeStampSeconds = lastTimeStampSeconds;

    // Updates the robot's odometry with april tags
    if (vision.canSeeAprilTags(limelight)) {
      currentTimeStampSeconds = vision.getTimeStampSeconds(limelight);

      double distanceFromClosestAprilTag = vision.getLimelightAprilTagDistance(limelight);

      // Depending on how many april tags we see, we change our confidence as more april tags
      // results in a much more accurate pose estimate
      if (vision.getNumberOfAprilTags(limelight) == 1) {
        double[] standardDeviations =
            oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        swerveDrive.setPoseEstimatorVisionConfidence(
            standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      } else if (vision.getNumberOfAprilTags(limelight) > 1) {
        double[] standardDeviations =
            twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        swerveDrive.setPoseEstimatorVisionConfidence(
            standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      }

      swerveDrive.addPoseEstimatorVisionMeasurement(
          vision.getPoseFromAprilTags(limelight),
          Logger.getTimestamp() - vision.getLatencySeconds(limelight));
    }

    
    Logger.recordOutput("Odometry/CurrentVisionPose" + limelight.getName(), vision.getPoseFromAprilTags(limelight));
    Logger.recordOutput("Odometry/CurrentCalculatePose", swerveDrive.getPose());

    lastTimeStampSeconds = currentTimeStampSeconds;
  }
}
